// The shape-learning sink, on the host. Two things are being protected here,
// and they are not the same thing.
//
// The first is arithmetic: every feature has one documented bin mapping in
// wiicam_learn.h, and a histogram read with the wrong mapping is not a
// slightly wrong measurement, it is a confident wrong answer. The mappings all
// CLAMP, never wrap, and the difference matters more than it looks: 64 & 31 is
// zero, so a blob four times the frame median would be filed as the darkest
// blob in the frame -- the opposite end of the very axis being measured.
//
// The second is the safety property, and it lives in wiicam_aim.cpp rather
// than in the sink: the positive class may only be fed from frames where the
// QUAD RESOLVER confirmed four real corners, because that label comes from
// where the blobs SIT and no size or shape gate has a vote in it. A sink that
// learned from the gate's own decisions and then taught the gate would drift
// once and never come back, silently. So the integration block below drives
// wiicam_aim_process_sz, not wl_note.
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include "wiicam_aim.h"
#include "wiicam_learn.h"
#include "aim_runtime.h"
#include "nvs.h"

// ---- fake NVS (same shape as wiicam_adapter_test's) -----------------------
// wiicam_aim.cpp reaches the store on begin() and on camsave; nothing here
// cares what it holds, only that the links resolve and the reads say "empty".
static unsigned char g_blob[512]; static size_t g_bl = 0; static bool g_bhave = false;
static unsigned char g_lens[64];  static size_t g_ll = 0; static bool g_lhave = false;
static bool is_lens(const char* k){ return k && k[0]=='l' && k[1]=='e' && k[2]=='n'; }
extern "C" {
esp_err_t nvs_open(const char*, nvs_open_mode_t, nvs_handle_t* h){ *h=1; return ESP_OK; }
esp_err_t nvs_set_blob(nvs_handle_t, const char* k, const void* v, size_t n){
    if (is_lens(k)) { memcpy(g_lens,v,n); g_ll=n; g_lhave=true; return ESP_OK; }
    memcpy(g_blob,v,n); g_bl=n; g_bhave=true; return ESP_OK; }
esp_err_t nvs_get_blob(nvs_handle_t, const char* k, void* o, size_t* l){
    if (is_lens(k)) {
        if (!g_lhave) return ESP_ERR_NVS_NOT_FOUND;
        if (*l < g_ll) return -1;
        memcpy(o, g_lens, g_ll); *l = g_ll; return ESP_OK;
    }
    if (!g_bhave) return ESP_ERR_NVS_NOT_FOUND;
    if (*l < g_bl) return -1;
    memcpy(o, g_blob, g_bl); *l = g_bl; return ESP_OK; }
static uint32_t g_u32 = 0; static bool g_uhave = false;
// Both gate keys by NAME. camreset erases the size gate's "gate0" AND the
// shape gate's "gate1", and a stub that only knows the first lets the second
// erase fall through to the calibration blob -- wiping a setting the command
// never mentions while leaving the one it did.
esp_err_t nvs_erase_key(nvs_handle_t, const char* k){
    if (k && (!strcmp(k, "gate0") || !strcmp(k, "gate1"))) g_uhave = false;
    else if (is_lens(k)) g_lhave = false; else g_bhave = false;
    return ESP_OK; }
esp_err_t nvs_set_i16(nvs_handle_t, const char*, int16_t){ return ESP_OK; }
esp_err_t nvs_get_i16(nvs_handle_t, const char*, int16_t*){ return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_set_u32(nvs_handle_t, const char*, uint32_t v){ g_u32=v; g_uhave=true; return ESP_OK; }
esp_err_t nvs_get_u32(nvs_handle_t, const char*, uint32_t* v){
    if (!g_uhave) return ESP_ERR_NVS_NOT_FOUND;
    *v = g_u32; return ESP_OK; }
esp_err_t nvs_commit(nvs_handle_t){ return ESP_OK; }
void nvs_close(nvs_handle_t){}
esp_err_t nvs_flash_init(void){ return ESP_OK; }
int64_t esp_timer_get_time(void){ return 0; }
}

static std::vector<std::string> g_lines, g_replies;
static void line_sink(const char* s){ g_lines.push_back(s); }
static void reply_sink(const char* s){ g_replies.push_back(s); }

static int fails = 0;
static void ck(bool ok, const char* m){ printf("  [%s] %s\n", ok?"PASS":"FAIL", m); if(!ok) fails++; }

// ---- reading the sink -----------------------------------------------------
// How many samples one class+feature histogram holds. -1 means the read was
// refused, which is a different answer from "empty" and must stay so.
static long hsum(int cls, int feat)
{
    const uint16_t* h = wl_hist(cls, feat);
    if (!h) return -1;
    long s = 0;
    for (int i = 0; i < WL_BINS; ++i) s += (long)h[i];
    return s;
}

static unsigned hat(int cls, int feat, int bin)
{
    const uint16_t* h = wl_hist(cls, feat);
    return (h && bin >= 0 && bin < WL_BINS) ? (unsigned)h[bin] : 0u;
}

// The single bin one sample landed in. -1 = the feature recorded nothing,
// -2 = it recorded in more than one place (which no single blob may do),
// -3 = the read was refused.
static int hbin(int cls, int feat)
{
    const uint16_t* h = wl_hist(cls, feat);
    if (!h) return -3;
    int found = -1;
    for (int i = 0; i < WL_BINS; ++i) {
        if (!h[i]) continue;
        if (found >= 0) return -2;
        found = i;
    }
    return found;
}

// One blob into an EMPTY sink: every assertion in the mapping block then reads
// a histogram holding exactly that blob and nothing else, so a stray count
// from a previous case cannot be mistaken for the one under test. The off/on
// pair is what forces the clear -- wl_enable only clears on the transition.
static void one(int sz, int bw, int bh, int intens, int imed, int flags)
{
    wl_enable(0);
    wl_enable(1);
    wl_note(0, sz, bw, bh, intens, imed, flags);
}

// ---- a fake full-mode bus transaction (as wiicam_adapter_test's) ----------
// Full mode is the only format carrying a box and an intensity, so it is the
// only one that can exercise five of the six features end to end. The vendored
// driver cannot read the format, so the unpack is ours and the transaction is
// a hook -- which is what makes the whole path reachable from the host.
struct FullObj { int x, y, sz, xmn, ymn, xmx, ymx, inten; };
static FullObj g_fobj[4];

static void full_pack(unsigned char* buf, unsigned char hdr, const FullObj* o)
{
    memset(buf, 0, WIICAM_FULL_LEN);
    buf[0] = hdr;
    for (int i = 0; i < 4; ++i) {
        unsigned char* f = buf + 1 + i * 9;
        f[0] = (unsigned char)(o[i].x & 0xFF);
        f[1] = (unsigned char)(o[i].y & 0xFF);
        f[2] = (unsigned char)((((o[i].y >> 8) & 3) << 6)
                             | (((o[i].x >> 8) & 3) << 4)
                             | (o[i].sz & 0x0F));
        f[3] = (unsigned char)o[i].xmn;  f[4] = (unsigned char)o[i].ymn;
        f[5] = (unsigned char)o[i].xmx;  f[6] = (unsigned char)o[i].ymx;
        f[7] = 0xA5;                     // reserved: nothing may read it
        f[8] = (unsigned char)o[i].inten;
    }
}

static int full_hook(unsigned char* buf, int len)
{
    if (len < WIICAM_FULL_LEN) return 0;   // a short read is a failed read
    full_pack(buf, 0x00, g_fobj);
    return 1;
}

// ---- driving one real frame through the front end -------------------------
static int      g_qpx[4] = {0,0,0,0}, g_qpy[4] = {0,0,0,0};
static int      g_qsz[4] = {-1,-1,-1,-1};
static unsigned g_qseen  = 0;
static uint64_t g_t      = 1000000;
static int      g_jit    = 0;
static const uint64_t DT = 4785;           // ~209 Hz, like the stock poll timer

// Fill the slots, poll them the way the firmware does, and hand the poll's own
// answer to the pipeline. The one pixel of jitter is not cosmetic: a
// byte-identical report is treated as the previous camera frame seen again and
// returns the cached answer without touching ANY stateful stage -- the
// learning sink included -- so an unjittered loop would measure one frame and
// report it as twenty.
static void frame(const FullObj* o)
{
    FullObj f[4];
    memcpy(f, o, sizeof(f));
    f[0].x += (g_jit & 1) ? 1 : -1;
    ++g_jit;
    memcpy(g_fobj, f, sizeof(g_fobj));
    float sx = 0.0f, sy = 0.0f;
    wiicam_aim_full_poll(g_qpx, g_qpy, g_qsz, &g_qseen);
    g_t += DT;
    wiicam_aim_process_sz(g_qpx, g_qpy, g_qsz, g_qseen, g_t, &sx, &sy);
}

// Arm a CLEAN capture through the serial surface. wl_enable clears on the
// off -> on EDGE, so anything that wants an empty sink has to make sure the
// capture was really off first; re-arming a running one deliberately leaves it
// alone, which the serial block below pins down on its own.
static void arm_clean(void)
{
    wiicam_cam_command("camlearn=on:0");
    wiicam_cam_command("camlearn=on:1");
}

// One counter out of '~camblob?', e.g. blobstat("br3=") -- the frames the
// resolver could only see three real corners in.
static unsigned long blobstat(const char* key)
{
    g_replies.clear();
    wiicam_cam_command("camblob?");
    unsigned long v = 0;
    if (!g_replies.empty()) {
        const char* p = strstr(g_replies[0].c_str(), key);
        if (p) sscanf(p + strlen(key), "%lu", &v);
    }
    return v;
}

// How many numbers a '~camlearn?' histogram line carries after its header.
// Counted rather than sscanf'd against 32 fields, because the failure worth
// catching is a line TRUNCATED by its own buffer: that loses bins silently off
// the tail, and the tail is where every clamped outlier is.
static int hist_line_bins(const std::string& ln)
{
    const size_t at = ln.find(" f=");
    if (at == std::string::npos) return -1;
    size_t i = ln.find(' ', at + 3);       // past the feature name
    if (i == std::string::npos) return -1;
    int n = 0;
    while (i < ln.size()) {
        while (i < ln.size() && ln[i] == ' ') ++i;
        if (i >= ln.size() || ln[i] < '0' || ln[i] > '9') break;
        ++n;
        while (i < ln.size() && ln[i] >= '0' && ln[i] <= '9') ++i;
    }
    return n;
}

int main()
{
    printf("wiicam learn\n\n");
    aim_runtime_begin();
    wiicam_aim_begin();
    wiicam_set_line_sink(line_sink);
    wiicam_set_reply_sink(reply_sink);
    wiicam_set_fullread_hook(full_hook);

    // ---- WL_SZ: bin = the 4-bit reported size ----------------------------
    {
        one(0, 4, 4, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_SZ) == 0, "size 0 is a real reported size and lands in bin 0");
        one(7, 4, 4, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_SZ) == 7, "the reported size IS the bin -- no scaling in between");
        one(15, 4, 4, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_SZ) == 15,
           "the largest size the 4-bit field can carry has its own bin, and it "
           "is not the last bin of the array -- bins 16..31 stay empty so a "
           "reader can see the feature only ever uses the bottom half");

        // A negative size means the sensor did not report one AT ALL (basic
        // format), not that it reported zero. Counted as zero it would put a
        // large sharp peak at the smallest possible blob -- a peak no LED ever
        // produced -- and that peak is exactly what a size window would then
        // be drawn around.
        one(-1, 4, 4, 100, 100, WL_HAS_BOX);
        ck(hsum(0, WL_SZ) == 0,
           "an unreported size records NOTHING, rather than a phantom blob at "
           "size 0");
        ck(hat(0, WL_SZ, 0) == 0u,
           "...and specifically not in bin 0, where it would read as the "
           "smallest blob the sensor can see");
        ck(wl_blobs(0) == 1u,
           "the blob still counts as a blob: the size is missing, the sample "
           "is not");
        ck(hbin(0, WL_BW) == 4 && hbin(0, WL_BH) == 4,
           "and the features it DOES carry are still recorded");
    }

    // ---- WL_BW / WL_BH: bin = min(side, 31) ------------------------------
    {
        one(3, 0, 9, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_BW) == 0 && hbin(0, WL_BH) == 9,
           "each box side goes to its own histogram, in native pixels");
        one(3, 31, 9, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_BW) == 31, "the last in-range width has the last bin");
        one(3, 200, 9, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_BW) == 31,
           "a width past the end of the array CLAMPS into the top bin, where "
           "an outlier is visible");
        ck(hat(0, WL_BW, 200 % WL_BINS) == 0u,
           "...and does not wrap: 200 folded modulo 32 is bin 8, a perfectly "
           "ordinary blob width, so the reader would never know it was there");
    }

    // ---- WL_ASPECT: bin = clamp(8*hi/lo, 8, 39) - 8 -----------------------
    // The feature the header expects to survive any bar geometry and any
    // viewing angle, so it is the one most likely to be read on its own.
    {
        one(3, 6, 6, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_ASPECT) == 0,
           "a perfectly square blob is bin 0 -- drop the -8 and every LED on "
           "the rig sits in bin 8 instead, in the middle of the range, and the "
           "whole feature reads as mildly elongated");
        one(3, 12, 6, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_ASPECT) == 8, "a 2:1 blob is bin 8");
        one(3, 6, 12, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_ASPECT) == 8,
           "and a 2:1 blob standing on end is the SAME bin -- the ratio is "
           "longer over shorter, so rolling the gun must not move the feature");
        one(3, 24, 6, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_ASPECT) == 24, "a 4:1 blob is bin 24, as the header says");

        // Worse than 4:1 has to pile up at the top of the array. 8:1 is bin 56
        // before the clamp, and 56 taken modulo 32 is 24 -- so a wrapping bump
        // would file the most elongated blob in the capture as exactly 4:1,
        // adding it to a bin that already means something else.
        one(3, 64, 8, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_ASPECT) == WL_BINS - 1,
           "an 8:1 streak clamps into the TOP bin, which is what 'or worse' "
           "has to mean when the array ends");
        ck(hat(0, WL_ASPECT, 24) == 0u,
           "...and lands nowhere near bin 24: a wrap would file a window blind "
           "as an ordinary 4:1 LED");

        // A zero side is not a flat blob. It is a blob one pixel across in
        // that axis once the sensor's 7-bit corners have been subtracted, and
        // there is no ratio to take. Forced to one, every small round LED
        // would be filed at the extremely-elongated end -- the wrong end, and
        // the end a shape gate would be drawn from.
        one(3, 0, 7, 100, 100, WL_HAS_BOX);
        ck(hsum(0, WL_ASPECT) == 0,
           "a box with a zero WIDTH records no aspect at all");
        ck(hat(0, WL_ASPECT, WL_BINS - 1) == 0u,
           "...and above all not in the top bin, the 'extremely elongated' end");
        one(3, 7, 0, 100, 100, WL_HAS_BOX);
        ck(hsum(0, WL_ASPECT) == 0, "a zero HEIGHT is refused the same way");
        ck(hbin(0, WL_BH) == 0 && hbin(0, WL_BW) == 7,
           "the sides themselves are still recorded -- only the ratio is "
           "withheld, because only the ratio is undefined");
    }

    // ---- WL_AREA: bin = min(w*h, 31) --------------------------------------
    {
        one(3, 4, 5, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_AREA) == 20, "area is the product of the two sides");
        one(3, 10, 10, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_AREA) == 31,
           "and a 100-pixel blob clamps into the top bin rather than reaching "
           "past the end of a 32-bin array");
        ck(hat(0, WL_AREA, 100 % WL_BINS) == 0u,
           "...not wrapping to bin 4, which is a blob the size of an LED");
    }

    // ---- WL_IREL: bin = clamp(16*i/median, 0, 31) -------------------------
    // Relative, because distance changes intensity by about 1/r^2: any
    // absolute threshold is wrong the moment the player steps back.
    {
        one(3, 4, 4, 200, 200, WL_HAS_BOX);
        ck(hbin(0, WL_IREL) == 16,
           "a blob at exactly the frame median is bin 16 -- the centre of the "
           "axis, which is the one value the whole feature is read against");
        one(3, 4, 4, 100, 200, WL_HAS_BOX);
        ck(hbin(0, WL_IREL) == 8, "half the median is bin 8");
        one(3, 4, 4, 400, 200, WL_HAS_BOX);
        ck(hbin(0, WL_IREL) == 31,
           "and twice the median is bin 32 before the clamp, so it lands in "
           "the top bin instead of one past the end of the array");
        one(3, 4, 4, 800, 200, WL_HAS_BOX);
        ck(hbin(0, WL_IREL) == 31, "four times the median clamps to the same top bin");
        ck(hat(0, WL_IREL, 0) == 0u,
           "...and NOT bin 0: 64 wrapped modulo 32 is zero, which would file "
           "the brightest blob in the frame as the dimmest one and invert the "
           "conclusion this histogram exists to support");
        one(3, 4, 4, 0, 200, WL_HAS_BOX);
        ck(hbin(0, WL_IREL) == 0, "a genuinely dark blob is a real bin 0");

        // A zero median is a frame with no measured brightness at all. The
        // division has no answer, and a guessed one would be a peak.
        one(3, 4, 4, 150, 0, WL_HAS_BOX);
        ck(hsum(0, WL_IREL) == 0,
           "a zero frame median SKIPS the relative intensity rather than "
           "dividing by it");
        ck(hbin(0, WL_SZ) == 3 && hbin(0, WL_BW) == 4,
           "...while every feature that does not need the median is still "
           "recorded, so one missing number costs one feature and not the blob");
    }

    // ---- extended mode: size only, and the rest left EMPTY ----------------
    // Without WL_HAS_BOX there is no box and no intensity to record. Filling
    // them with the zeroes that happen to be in the arguments would read as a
    // real measurement of a round, area-zero, pitch-dark blob -- a huge sharp
    // peak at bin 0 of three features at once, which is far worse than a gap.
    {
        one(9, 4, 5, 200, 100, 0);
        ck(hbin(0, WL_SZ) == 9, "extended mode still feeds the one feature it has");
        ck(hsum(0, WL_BW) == 0 && hsum(0, WL_BH) == 0,
           "and records no box width or height -- there is no box");
        ck(hsum(0, WL_AREA) == 0,
           "no area, which would otherwise be a false peak at bin 0 on every "
           "blob the gun ever sees in extended");
        ck(hsum(0, WL_ASPECT) == 0, "no aspect");
        ck(hsum(0, WL_IREL) == 0,
           "and no relative intensity, even though a median was supplied");
        ck(wl_blobs(0) == 1u,
           "the blob is still counted, so the report cannot claim a full-mode "
           "sample count for a capture run in extended");
    }

    // ---- saturation -------------------------------------------------------
    // A bin that wraps past 65535 turns the tallest peak in the distribution
    // into the shortest and inverts the conclusion the histogram exists to
    // support. A capture long enough to saturate has already made its point.
    {
        wl_enable(0);
        wl_enable(1);
        for (long i = 0; i < 65535; ++i) wl_note(0, 5, 0, 0, 0, 0, 0);
        ck(hat(0, WL_SZ, 5) == 65535u, "a bin fills to 65535");
        wl_note(0, 5, 0, 0, 0, 0, 0);
        ck(hat(0, WL_SZ, 5) == 65535u,
           "and the next sample leaves it there rather than wrapping to zero, "
           "which would make the commonest blob in the capture look like the "
           "rarest");
        for (int i = 0; i < 10; ++i) wl_note(0, 5, 0, 0, 0, 0, 0);
        ck(hat(0, WL_SZ, 5) == 65535u, "it stays pinned, sample after sample");
        ck(wl_blobs(0) == 65546u,
           "the blob COUNT keeps rising past a saturated bin -- it is 32 bits "
           "and it is how a reader knows the bin is pinned rather than exact");
        ck(hat(0, WL_SZ, 4) == 0u && hat(0, WL_SZ, 6) == 0u,
           "and a saturating bin does not spill into its neighbours");
    }

    // ---- the class label ---------------------------------------------------
    // wiicam_aim.cpp hands the sink `keep[i] ? 0 : 1`. Swap those and the two
    // distributions are exchanged wholesale: the report would claim the gate
    // keeps precisely what it throws away, and two well separated classes
    // would still look well separated, so nothing about the shape of the
    // answer would give the mistake away.
    {
        wl_enable(0);
        wl_enable(1);
        wl_note(0, 2, 4, 4, 100, 100, WL_HAS_BOX);
        wl_note(1, 11, 20, 5, 100, 100, WL_HAS_BOX);
        ck(hbin(0, WL_SZ) == 2 && hbin(1, WL_SZ) == 11,
           "the two classes are separate histograms and a blob lands in the "
           "one it was labelled with");
        ck(wl_blobs(0) == 1u && wl_blobs(1) == 1u,
           "and each class counts only its own blobs");
        ck(hbin(0, WL_ASPECT) == 0 && hbin(1, WL_ASPECT) == 24,
           "every feature is per class, not just the sample count -- one wide "
           "shared histogram is the failure that makes two distributions look "
           "like one");

        // Out of range is refused, not folded into a valid class.
        wl_enable(0);
        wl_enable(1);
        wl_note(WL_CLASSES, 3, 4, 4, 100, 100, WL_HAS_BOX);
        wl_note(-1, 3, 4, 4, 100, 100, WL_HAS_BOX);
        ck(wl_blobs(0) == 0u && wl_blobs(1) == 0u
           && hsum(0, WL_SZ) == 0 && hsum(1, WL_SZ) == 0,
           "a blob with an out-of-range class is dropped, not written past the "
           "end of a 2x6x32 array");
    }

    // ---- the enable semantics ----------------------------------------------
    {
        wl_enable(0);
        wl_enable(1);
        ck(wl_enabled() == 1, "wl_enable(1) arms the capture");
        wl_note(0, 3, 4, 4, 100, 100, WL_HAS_BOX);
        wl_note_frame();
        ck(wl_blobs(0) == 1u && wl_frames() == 1u,
           "an armed sink counts the blob and the frame it came from");

        // Arming CLEARS. A distribution accumulated across a change of
        // sensitivity or of the LED bar is two rigs averaged into one wide
        // spread, and a wide spread reads as "these cannot be separated" when
        // the truth is that neither rig was ever measured.
        wl_enable(0);
        wl_enable(1);
        ck(hsum(0, WL_SZ) == 0 && wl_blobs(0) == 0u && wl_frames() == 0u,
           "arming the capture CLEARS what was there -- histogram, blob counts "
           "and frame count together");
        ck(hsum(1, WL_IREL) == 0 && wl_blobs(1) == 0u,
           "...both classes and every feature, not just the one being read");

        wl_note(0, 3, 4, 4, 100, 100, WL_HAS_BOX);
        wl_note_frame();
        wl_enable(0);
        ck(wl_enabled() == 0, "wl_enable(0) stops the capture");
        wl_note(0, 7, 9, 9, 100, 100, WL_HAS_BOX);
        wl_note_frame();
        ck(wl_blobs(0) == 1u && wl_frames() == 1u && hat(0, WL_SZ, 7) == 0u,
           "a stopped sink accumulates NOTHING -- not the blob, not the frame, "
           "not the bin");
        ck(hat(0, WL_SZ, 3) == 1u,
           "...and leaves what it already holds alone, so the capture can "
           "still be read out after it is stopped");

        // Clearing without disarming: the way to start a second capture on the
        // same rig without a round trip through off.
        wl_note_frame();                         // still stopped: no effect
        wl_enable(1);                            // off -> on, so this clears too
        wl_note(0, 6, 4, 4, 100, 100, WL_HAS_BOX);
        wl_note_frame();
        wl_reset();
        ck(wl_enabled() == 1,
           "wl_reset clears without DISARMING -- a reset that stopped the "
           "capture would silently end a session the user thinks is running");
        ck(hsum(0, WL_SZ) == 0 && wl_blobs(0) == 0u && wl_frames() == 0u,
           "...and it really does clear all three counters");
        wl_note(0, 6, 4, 4, 100, 100, WL_HAS_BOX);
        ck(wl_blobs(0) == 1u, "...and the sink keeps accumulating afterwards");
    }

    // ---- out-of-range read-out ---------------------------------------------
    // The report walks classes and features in a loop. A read that answered
    // with the memory next to the array would print 32 plausible numbers for a
    // class that does not exist.
    {
        ck(wl_hist(-1, WL_SZ) == 0 && wl_hist(WL_CLASSES, WL_SZ) == 0,
           "an out-of-range class returns null, not the bytes beside the array");
        ck(wl_hist(0, -1) == 0 && wl_hist(0, WL_NFEAT) == 0,
           "and so does an out-of-range feature");
        ck(wl_hist(0, WL_SZ) != 0 && wl_hist(WL_CLASSES - 1, WL_NFEAT - 1) != 0,
           "while both ends of the real range still answer");
        ck(wl_blobs(-1) == 0u && wl_blobs(WL_CLASSES) == 0u,
           "an out-of-range class has no blob count either");
        ck(!strcmp(wl_feat_name(-1), "?") && !strcmp(wl_feat_name(WL_NFEAT), "?"),
           "an out-of-range feature name is '?', not a read off the end of the "
           "name table");
        ck(!strcmp(wl_feat_name(WL_SZ), "sz")
           && !strcmp(wl_feat_name(WL_BW), "bw")
           && !strcmp(wl_feat_name(WL_BH), "bh")
           && !strcmp(wl_feat_name(WL_ASPECT), "aspect")
           && !strcmp(wl_feat_name(WL_AREA), "area")
           && !strcmp(wl_feat_name(WL_IREL), "irel"),
           "the six names match the enum order -- the tools index the report "
           "by name, so a shifted table mislabels every histogram");
    }

    // ======================================================================
    // THE INTEGRATION. Everything below is driven through
    // wiicam_aim_process_sz, because the safety property is not in the sink.
    // ======================================================================

    // The rig: four LEDs on a rectangle, each with its own size, box and
    // intensity so a feature landing in the wrong bin says which blob it came
    // from. Boxes are in the sensor's native 128x96 array (7-bit corners),
    // which is what the box features are documented in.
    //
    //  slot 0  4x4 box, area 16, square      size 2  intensity  60
    //  slot 1  6x3 box, area 18, 2:1         size 3  intensity  90
    //  slot 2  8x2 box, area 16, 4:1         size 4  intensity 110
    //  slot 3  5x5 box, area 25, square      size 5  intensity 190
    //
    // The four intensities are chosen so that no other plausible centre gives
    // the same answer: the median of an even count is (90+110)/2 = 100, while
    // the upper middle value alone is 110 and the arithmetic mean is 112, and
    // each of the three files these four blobs in a DIFFERENT set of bins.
    // Against 100 the relative intensities are 9, 14, 17 and 30 sixteenths.
    static const FullObj RIG[4] = {
    //    x    y  sz  xmn ymn xmx ymx  inten
        { 256, 240, 2,  10, 20, 14, 24,  60 },
        { 768, 240, 3,  10, 20, 16, 23,  90 },
        { 256, 528, 4,   0,  0,  8,  2, 110 },
        { 768, 528, 5,   1,  1,  6,  6, 190 },
    };

    wiicam_cam_command("cam=res:2,dash:0,mirx:1,bmin:0,bmax:15,rtol:0");
    wiicam_cam_command("cam=fmt:2");

    // ---- nothing at all while the capture is off ---------------------------
    {
        wiicam_cam_command("camlearn=on:0");     // stop...
        wiicam_cam_command("camlearn=reset");    // ...and start from empty
        const unsigned long r4a = blobstat("br4=");
        for (int i = 0; i < 6; ++i) frame(RIG);
        const unsigned long r4b = blobstat("br4=");
        ck(r4b >= r4a + 6,
           "six frames really did reach the resolver with all four corners "
           "seen -- the case that WOULD be learned from if the capture were on");
        ck(wl_frames() == 0u && wl_blobs(0) == 0u && wl_blobs(1) == 0u,
           "and with the capture off not one of them was counted: the sink is "
           "inert until it is asked for, so a shipped gun measures nothing");
        ck(hsum(0, WL_SZ) == 0 && hsum(0, WL_IREL) == 0,
           "...and no histogram moved either");
    }

    // ---- a rejection where the missing LED should be teaches nothing --------
    // This is the whole safety property. A negative label is a positional
    // verdict: the reconstructed fourth corner says where the missing LED must
    // be, and a rejected blob nowhere near it was not an LED. A rejected blob
    // sitting ON it is the opposite reading -- the gate almost certainly took
    // a real corner -- and learning from that is the feedback loop the header
    // refuses: the gate teaching itself that LEDs are strays, drifting once
    // and never coming back, with a working-looking gun the whole time.
    //
    // Here the rejected blob IS a rig corner, so every one of these frames
    // reaches the negative branch and every one of them has to come out of it
    // with nothing recorded. ("The NEGATIVE class, and the second way into it"
    // further down is the same branch with the blob moved off the corner,
    // which is the case that does produce a sample.)
    //
    // Worth recording what this arrangement demonstrates about the sensor: the
    // wiicam has four object slots, so a rejected blob costs the resolver its
    // fourth REAL corner. On this hardware r.n_real == 4 therefore implies
    // every blob was kept -- see the notes at the end of this file.
    {
        arm_clean();
        wiicam_cam_command("cam=bmin:0,bmax:5");
        FullObj junk[4];
        memcpy(junk, RIG, sizeof(junk));
        junk[3].sz = 12;                     // a big diffuse patch, out of window
        const unsigned long rej_a  = blobstat("brej=");
        const unsigned long r3a    = blobstat("br3=");
        const unsigned long near_a = blobstat("bnear=");
        for (int i = 0; i < 6; ++i) frame(junk);
        const unsigned long rej_b  = blobstat("brej=");
        const unsigned long r3b    = blobstat("br3=");
        ck(rej_b >= rej_a + 6,
           "the size window really did reject a blob in every one of these "
           "frames -- without that this block proves nothing");
        ck(r3b >= r3a + 6,
           "...and every one of them left the resolver with three real "
           "corners, not four");
        ck(blobstat("bnear=") >= near_a + 6,
           "...and every rejected blob was sitting where the reconstruction "
           "says the missing LED is, which is the false-negative meter and the "
           "one reading that must never become a negative SAMPLE");
        ck(wl_frames() == 0u,
           "a frame the resolver could not confirm at four corners contributes "
           "NO frame, however many blobs it had");
        ck(wl_blobs(0) == 0u && wl_blobs(1) == 0u,
           "and no blobs from it, in either class: a blob rejected on top of "
           "the missing corner is evidence about the GATE, not about strays, "
           "and the gate must never end up teaching itself");
        ck(hsum(0, WL_SZ) == 0 && hsum(1, WL_SZ) == 0,
           "...so not one bin moved");
        wiicam_cam_command("cam=bmin:0,bmax:15");
    }

    // ---- a confirmed frame, feature by feature -----------------------------
    {
        for (int i = 0; i < 4; ++i) frame(RIG);   // re-lock with the window open
        arm_clean();                              // empty, so exactly one frame
        const unsigned long r4a = blobstat("br4=");
        frame(RIG);
        ck(blobstat("br4=") == r4a + 1, "one confirmed frame, and only one");
        ck(wl_frames() == 1u,
           "a confirmed frame counts once as a frame -- the divisor the report "
           "prints its blob counts against");
        ck(wl_blobs(0) == 4u,
           "all four kept blobs land in class 0, the resolver-confirmed LEDs");
        ck(wl_blobs(1) == 0u,
           "and nothing lands in class 1: nothing was rejected in this frame");

        ck(hat(0, WL_SZ, 2) == 1u && hat(0, WL_SZ, 3) == 1u
           && hat(0, WL_SZ, 4) == 1u && hat(0, WL_SZ, 5) == 1u
           && hsum(0, WL_SZ) == 4,
           "the four reported sizes reach the size histogram, one each");

        // Each blob wears its OWN box, and the two sides do not swap. (The
        // poll fills its box arrays by hardware slot while the learning loop
        // walks the compacted seen list, but those two indexings can only
        // diverge on a frame with an empty slot, and such a frame is never
        // confirmed at four corners -- see the notes at the end of this file.
        // So what these two checks pin is the transposition, not the gap.)
        ck(hat(0, WL_BW, 4) == 1u && hat(0, WL_BW, 6) == 1u
           && hat(0, WL_BW, 8) == 1u && hat(0, WL_BW, 5) == 1u
           && hsum(0, WL_BW) == 4,
           "each blob's own box WIDTH is recorded, in native pixels");
        ck(hat(0, WL_BH, 4) == 1u && hat(0, WL_BH, 3) == 1u
           && hat(0, WL_BH, 2) == 1u && hat(0, WL_BH, 5) == 1u
           && hsum(0, WL_BH) == 4,
           "and its own box HEIGHT -- not the width again, which would make "
           "every blob on the rig look perfectly square");

        ck(hat(0, WL_ASPECT, 0) == 2u && hat(0, WL_ASPECT, 8) == 1u
           && hat(0, WL_ASPECT, 24) == 1u && hsum(0, WL_ASPECT) == 4,
           "the two square blobs, the 2:1 and the 4:1 each reach the aspect "
           "bin the header documents for them");
        ck(hat(0, WL_AREA, 16) == 2u && hat(0, WL_AREA, 18) == 1u
           && hat(0, WL_AREA, 25) == 1u && hsum(0, WL_AREA) == 4,
           "and the areas are the products of each blob's own two sides");

        // The median is this frame's own, over the KEPT blobs, and there are
        // four of them -- an even count, so it is the mean of the middle two.
        // Take the upper middle value instead (110) and these four blobs land
        // in 8, 13, 16, 27; take the arithmetic mean (112) and they land in
        // 8, 12, 15, 27. Neither is 9, 14, 17, 30.
        ck(hat(0, WL_IREL, 9) == 1u && hat(0, WL_IREL, 14) == 1u
           && hat(0, WL_IREL, 17) == 1u && hat(0, WL_IREL, 30) == 1u
           && hsum(0, WL_IREL) == 4,
           "intensity is filed against the MEDIAN of this frame's kept blobs, "
           "and for an even count that is the mean of the middle two: "
           "60/90/110/190 against 100 is bins 9, 14, 17, 30");

        // Same rig, same ratios, half the light -- a player one step further
        // back. Every bin must be identical, because the feature is relative.
        // An absolute threshold would move all four.
        FullObj dim[4];
        memcpy(dim, RIG, sizeof(dim));
        for (int i = 0; i < 4; ++i) dim[i].inten = RIG[i].inten / 2;
        frame(dim);
        ck(wl_frames() == 2u && wl_blobs(0) == 8u, "a second confirmed frame");
        ck(hat(0, WL_IREL, 9) == 2u && hat(0, WL_IREL, 14) == 2u
           && hat(0, WL_IREL, 17) == 2u && hat(0, WL_IREL, 30) == 2u
           && hsum(0, WL_IREL) == 8,
           "30/45/55/95 lands in exactly the same four bins as 60/90/110/190 "
           "-- the feature is RELATIVE, so stepping back from the bar must not "
           "shift the distribution");
    }

    // ---- extended mode feeds size and nothing else -------------------------
    // In extended there is no box and no intensity to feed, and the flag that
    // says so is the only thing standing between the report and three
    // histograms full of zero-bin blobs that were never measured.
    {
        wiicam_cam_command("cam=fmt:1");
        int epx[4] = {256, 768, 256, 768};
        int epy[4] = {240, 240, 528, 528};
        int esz[4] = {2, 3, 4, 5};
        float sx = 0.0f, sy = 0.0f;
        for (int i = 0; i < 4; ++i) {         // re-lock in the new format
            epx[0] += (i & 1) ? 1 : -1;
            g_t += DT;
            wiicam_aim_process_sz(epx, epy, esz, 0xF, g_t, &sx, &sy);
        }
        arm_clean();
        const unsigned long r4a = blobstat("br4=");
        for (int i = 0; i < 3; ++i) {
            epx[0] += (i & 1) ? 1 : -1;
            g_t += DT;
            wiicam_aim_process_sz(epx, epy, esz, 0xF, g_t, &sx, &sy);
        }
        ck(blobstat("br4=") == r4a + 3, "three confirmed extended frames");
        ck(wl_frames() == 3u && wl_blobs(0) == 12u,
           "extended frames are learned from -- the size is the one feature "
           "the format carries and it is the feature the gate uses");
        ck(hat(0, WL_SZ, 2) == 3u && hat(0, WL_SZ, 5) == 3u
           && hsum(0, WL_SZ) == 12,
           "and the sizes are all there");
        ck(hsum(0, WL_BW) == 0 && hsum(0, WL_BH) == 0 && hsum(0, WL_AREA) == 0
           && hsum(0, WL_ASPECT) == 0 && hsum(0, WL_IREL) == 0,
           "while every box-derived feature stays EMPTY: a run in extended "
           "must not leave twelve phantom round, area-zero blobs behind for a "
           "reader to draw a shape gate around");
        wiicam_cam_command("cam=fmt:2");
    }

    // ---- the NEGATIVE class, and the second way into it ---------------------
    // A rejected blob is only evidence when the frame around it was labelled
    // by GEOMETRY. The resolver supplies that label from three real corners
    // and a reconstructed fourth: the reconstruction says where the missing
    // LED must be, so a rejected blob sitting nowhere near ANY corner was not
    // an LED whatever it looked like. That is the one path into class 1, and
    // the SHAPE gate is now a second way to walk it -- the gate that rejected
    // the blob is not part of the label, so a shape rejection has to feed the
    // negative class exactly as a size-window rejection does. If it did not,
    // the one gate with a real measurement behind it would be the one gate
    // that never contributed a sample to the distribution it exists to be
    // chosen from.
    //
    // Two frames on the same rig. SHAPE_RIG is four square corners, which is
    // what the resolver locks on; SHAPE_STRAY keeps its first three and puts a
    // stray in the fourth slot, at the middle of the rig -- 68 px from every
    // corner in 240-space, comfortably past the DOUBLED association radius the
    // negative label demands, so it cannot be the LED that went missing.
    //
    //   kept    4x4, 5x5 and 3x3, all square, sizes 2/3/4, px 60/100/140
    //   stray   5x2, which is 2.5:1 and past a 2:1 armax, size 7, px 150
    //
    // The stray's six features land in six distinct bins and not one of them
    // clamps, so a feature recorded in the wrong place says which one it was.
    static const FullObj SHAPE_RIG[4] = {
    //    x    y  sz  xmn ymn xmx ymx  px
        { 256, 240, 2,  10, 20, 14, 24,  60 },
        { 768, 240, 3,  10, 20, 15, 25, 100 },
        { 256, 528, 4,  10, 20, 13, 23, 140 },
        { 768, 528, 5,  10, 20, 14, 24, 190 },
    };
    static const FullObj SHAPE_STRAY[4] = {
    //    x    y  sz  xmn ymn xmx ymx  px
        { 256, 240, 2,  10, 20, 14, 24,  60 },
        { 768, 240, 3,  10, 20, 15, 25, 100 },
        { 256, 528, 4,  10, 20, 13, 23, 140 },
        { 512, 384, 7,  10, 20, 15, 22, 150 },   // 5 wide, 2 tall, at the centre
    };
    // The same three corners and the same stray SIZE and INTENSITY, with a box
    // the height cut is the only gate that objects to: 3 wide and 9 tall is
    // 3:1, so a 2:1 armax would catch it too and there would be no telling
    // which knob did the rejecting -- bsrej counts them all in one place.
    // Kept at 9 rather than higher so none of its four box features clamps:
    // width 3, height 9, aspect 24 eighths and area 27 all land in bins of
    // their own, and none of them is a bin the 5x2 stray above used.
    static const FullObj SHAPE_STRAY_TALL[4] = {
    //    x    y  sz  xmn ymn xmx ymx  px
        { 256, 240, 2,  10, 20, 14, 24,  60 },
        { 768, 240, 3,  10, 20, 15, 25, 100 },
        { 256, 528, 4,  10, 20, 13, 23, 140 },
        { 512, 384, 7,  10, 20, 13, 29, 150 },   // 3 wide, 9 tall, at the centre
    };
    {
        // Lock on four real corners first, with every gate open and the
        // capture off, so nothing here is learned from the lock-in.
        wiicam_cam_command("cam=fmt:2,bmin:0,bmax:15,rtol:0,bhmax:0,pxmax:0,armax:0");
        wiicam_cam_command("camlearn=on:0");
        for (int i = 0; i < 8; ++i) frame(SHAPE_RIG);

        wiicam_cam_command("cam=armax:16");
        arm_clean();
        const unsigned long srej_a = blobstat("bsrej=");
        const unsigned long rej_a  = blobstat("brej=");
        const unsigned long r3a    = blobstat("br3=");
        const unsigned long far_a  = blobstat("bfar=");
        frame(SHAPE_STRAY);
        ck(blobstat("bsrej=") == srej_a + 1 && blobstat("brej=") == rej_a,
           "the SHAPE gate is what rejected the stray -- its 2.5:1 box, not "
           "its size, which the window was wide open for");
        ck(blobstat("br3=") == r3a + 1,
           "...and the three corners left behind still resolved, with the "
           "fourth reconstructed: without that there is no positional label "
           "and nothing may be learned at all");
        ck(blobstat("bfar=") == far_a + 1,
           "...and the stray sat nowhere near the reconstructed corner, which "
           "is what makes it a stray rather than a corner the gate stole");

        ck(wl_blobs(1) == 1u,
           "a SHAPE-gate rejection reaches the negative class: the label came "
           "from the resolver, so which gate did the rejecting cannot matter");
        ck(wl_blobs(0) == 0u && wl_frames() == 0u,
           "and nothing reaches the positive class from that frame -- three "
           "real corners is not four, and only a four-corner frame says every "
           "blob in it was an LED");

        ck(hbin(1, WL_SZ) == 7 && hbin(1, WL_BW) == 5 && hbin(1, WL_BH) == 2,
           "the negative sample carries the STRAY's own size and box, not the "
           "first slot's or the last kept blob's");
        ck(hbin(1, WL_ASPECT) == 12 && hbin(1, WL_AREA) == 10,
           "...and its own aspect and area, 2.5:1 over 10 native pixels -- the "
           "shape that got it rejected is the shape that gets recorded");
        // Three kept blobs, so the median is the ODD branch: the middle value
        // itself, 100, not the mean of the middle two. Take the even branch
        // here and 150 is filed against (100+140)/2 = 120, which is bin 20.
        ck(hbin(1, WL_IREL) == 24,
           "and its intensity is filed against the median of the three blobs "
           "that were KEPT -- 150 against 100 is bin 24, where averaging the "
           "middle pair of an odd count would put it at 20");
        ck(hsum(0, WL_SZ) == 0 && hsum(0, WL_IREL) == 0,
           "...with the positive histograms still completely empty, so the "
           "sample cannot have been filed under both labels");

        // The same frame, the same stray, rejected by the SIZE WINDOW instead.
        // Two gates, one negative class: the sample they produce has to be the
        // same sample, or the distribution a gate is chosen from depends on
        // which gate was switched on while it was captured.
        wiicam_cam_command("cam=armax:0,bmin:0,bmax:5");
        const unsigned long srej_b = blobstat("bsrej=");
        const unsigned long rej_b  = blobstat("brej=");
        frame(SHAPE_STRAY);
        ck(blobstat("brej=") == rej_b + 1 && blobstat("bsrej=") == srej_b,
           "this time the size WINDOW rejected it -- size 7 outside 0..5, with "
           "the shape gate off");
        ck(wl_blobs(1) == 2u && wl_blobs(0) == 0u,
           "...and it is the second sample in the negative class");
        ck(hat(1, WL_SZ, 7) == 2u && hat(1, WL_BW, 5) == 2u
           && hat(1, WL_BH, 2) == 2u && hat(1, WL_ASPECT, 12) == 2u
           && hat(1, WL_AREA, 10) == 2u && hat(1, WL_IREL, 24) == 2u,
           "landing in the same six bins as the shape-gate rejection did: the "
           "sink is told what the blob looked like, never which gate objected "
           "to it");

        // And a THIRD way in: the height cut. It is a different knob inside
        // the same shape gate, and it is the one with a real capture behind
        // it, so it had better be able to contribute to the distribution it
        // will be re-chosen from. A height rejection that fed nothing would
        // leave the negative class describing only the strays the OTHER knobs
        // happen to catch -- which is a distribution shaped by the gate that
        // was switched on while it was captured, and that is precisely what
        // labelling by geometry exists to avoid.
        wiicam_cam_command("cam=bmin:0,bmax:15,bhmax:8,pxmax:0,armax:0");
        const unsigned long srej_c = blobstat("bsrej=");
        const unsigned long rej_c  = blobstat("brej=");
        const unsigned long r3c    = blobstat("br3=");
        const unsigned long far_c  = blobstat("bfar=");
        frame(SHAPE_STRAY_TALL);
        ck(blobstat("bsrej=") == srej_c + 1 && blobstat("brej=") == rej_c,
           "the HEIGHT cut is what rejected this one -- its 9-row box, with "
           "the size window wide open and the other two shape knobs off");
        ck(blobstat("br3=") == r3c + 1 && blobstat("bfar=") == far_c + 1,
           "...three corners resolved with the fourth reconstructed, and the "
           "stray nowhere near it: the positional label the negative class "
           "needs, arrived at without the height cut having any vote in it");
        ck(wl_blobs(1) == 3u && wl_blobs(0) == 0u,
           "a bhmax rejection reaches the negative class too -- the third of "
           "the three ways in, and the sink cannot tell them apart");
        ck(hat(1, WL_BH, 9) == 1u && hat(1, WL_BW, 3) == 1u,
           "carrying the tall stray's OWN 3x9 box: 9 rows in the height "
           "histogram is the sample a reader needs to see where a height cut "
           "could go, and it is the feature that got the blob rejected");
        ck(hat(1, WL_ASPECT, 16) == 1u && hat(1, WL_AREA, 27) == 1u,
           "...with its own 3:1 aspect and its 27 pixels of area, none of them "
           "in a bin the 5x2 stray used, so a sample filed under the wrong "
           "blob's box would be visible rather than absorbed into a count "
           "that was already there");
        ck(hat(1, WL_SZ, 7) == 3u && hat(1, WL_IREL, 24) == 3u,
           "...and its size and relative intensity land in the same two bins "
           "all three rejections did: the two features that did not change are "
           "the two that say the sink was told what the blob looked like and "
           "never which gate objected to it");
        ck(hat(1, WL_BH, 2) == 2u,
           "and the earlier pair are still where they were -- the third sample "
           "is added to the negative class, not written over it");

        wiicam_cam_command("cam=bmin:0,bmax:15,rtol:0,bhmax:0,pxmax:0,armax:0");
    }

    // ======================================================================
    // The serial surface: '~camlearn...' in wiicam_cam_command.
    // ======================================================================
    {
        // The previous capture is still armed and still holding samples: that
        // is what the arming command has to get rid of.
        for (int i = 0; i < 3; ++i) frame(RIG);
        wiicam_cam_command("camlearn=on:0");
        ck(wl_frames() > 0u && hsum(0, WL_SZ) > 0,
           "an earlier capture is sitting in the sink, waiting to be read");

        g_replies.clear();
        ck(wiicam_cam_command("camlearn=on:1"), "'~camlearn=on:1' is handled");
        ck(g_replies.size() == 1
           && g_replies[0] == "CAM: learn ON -- feeding only "
                              "resolver-confirmed frames\n",
           "...and answers with one line naming the one thing that makes the "
           "capture trustworthy");
        ck(wl_enabled() == 1, "...and arms the capture");
        ck(wl_frames() == 0u && wl_blobs(0) == 0u && hsum(0, WL_SZ) == 0,
           "...having cleared the previous one first, so a distribution is "
           "never two rigs averaged into one wide spread -- which would read "
           "as 'cannot be separated' when neither rig was measured");

        frame(RIG);
        frame(RIG);
        ck(wl_frames() == 2u && wl_blobs(0) == 8u, "two frames into the capture");

        // The clear is on the off -> on EDGE. Re-sending on:1 to a capture
        // that is already running leaves it alone, and has to: the tools
        // re-send a key on every keystroke, and a keystroke that silently
        // discarded a running session would be the worse failure of the two.
        wiicam_cam_command("camlearn=on:1");
        ck(wl_frames() == 2u && wl_blobs(0) == 8u && wl_enabled() == 1,
           "re-arming a capture that is already running does NOT restart it -- "
           "the clear belongs to the off -> on edge, not to every on:1");

        g_replies.clear();
        ck(wiicam_cam_command("camlearn=reset"), "'~camlearn=reset' is handled");
        ck(g_replies.size() == 1 && g_replies[0] == "CAM: learn cleared\n",
           "...and says so in one line");
        ck(wl_enabled() == 1,
           "...WITHOUT disarming: a reset that stopped the capture would end a "
           "session the user believes is still running");
        ck(wl_frames() == 0u && wl_blobs(0) == 0u && hsum(0, WL_SZ) == 0,
           "...and the histograms really are empty again");
        frame(RIG);
        ck(wl_frames() == 1u, "...and the capture goes straight on accumulating");

        g_replies.clear();
        ck(wiicam_cam_command("camlearn=on:0"), "'~camlearn=on:0' is handled");
        ck(g_replies.size() == 1
           && g_replies[0] == "CAM: learn off -- feeding only "
                              "resolver-confirmed frames\n",
           "...and reports the state it actually reached, not the one asked "
           "for");
        ck(wl_enabled() == 0, "...and stops the capture");
        frame(RIG);
        frame(RIG);
        ck(wl_frames() == 1u,
           "...for real: frames after the stop are not counted");
        ck(hsum(0, WL_SZ) == 4,
           "...while what was already measured is still there to be read out");
    }

    // ---- the report --------------------------------------------------------
    {
        arm_clean();
        frame(RIG);
        frame(RIG);
        frame(RIG);
        g_replies.clear();
        ck(wiicam_cam_command("camlearn?"), "'~camlearn?' is handled");
        ck(g_replies.size() == 1 + WL_CLASSES * WL_NFEAT,
           "the report is a summary line plus exactly one line per class per "
           "feature -- twelve of them, split so a tool that loses one line "
           "loses one feature rather than the whole capture");

        char sum[160];
        snprintf(sum, sizeof(sum),
                 "CAM: learn on=1 frames=3 led=12 rej=0 bins=%d\n", WL_BINS);
        ck(!g_replies.empty() && g_replies[0] == sum,
           "the summary carries the armed flag, the frame count, both class "
           "counts and the bin count -- everything needed to read the twelve "
           "lines that follow");

        static const char* const ORDER[WL_NFEAT] =
            { "sz", "bw", "bh", "aspect", "area", "irel" };
        bool shape_ok = g_replies.size() == 1 + WL_CLASSES * WL_NFEAT;
        bool bins_ok  = shape_ok;
        for (int cls = 0; cls < WL_CLASSES && shape_ok; ++cls) {
            for (int f = 0; f < WL_NFEAT; ++f) {
                char head[64];
                snprintf(head, sizeof(head), "CAM: hist c=%d f=%s ", cls, ORDER[f]);
                const std::string& ln = g_replies[1 + cls * WL_NFEAT + f];
                if (ln.compare(0, strlen(head), head) != 0) { shape_ok = false; break; }
                if (hist_line_bins(ln) != WL_BINS) bins_ok = false;
            }
        }
        ck(shape_ok,
           "each line names its class and its feature, in enum order -- the "
           "tools match on that header, and an unlabelled or reordered dump is "
           "twelve rows of numbers nobody can attribute");
        ck(bins_ok,
           "and each carries all 32 bins: a line truncated by its own buffer "
           "loses the tail, which is exactly where every clamped outlier is");

        // The numbers on the line have to be the histogram, not a restatement
        // of it. Three identical confirmed frames put three blobs in size bin
        // 2 and none in bin 0.
        unsigned b[WL_BINS];
        memset(b, 0, sizeof(b));
        int got = 0;
        if (g_replies.size() > 1)
            got = sscanf(g_replies[1].c_str(),
                         "CAM: hist c=0 f=sz %u %u %u %u %u %u",
                         &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]);
        ck(got == 6 && b[0] == 0u && b[2] == 3u && b[3] == 3u
           && b[4] == 3u && b[5] == 3u,
           "and the bins on the line are the bins in the sink");
    }

    // ---- the report at its WIDEST -------------------------------------------
    // The line above carried single-digit counts, which is the easy case. A
    // capture left running is the real one: five-digit counts in all 32 bins of
    // the longest-named feature is the widest line this report can ever emit,
    // and it is the line a too-small buffer silently truncates. Truncation
    // takes the TAIL, which is where every clamped outlier in the capture sits
    // -- so the bins that go missing are exactly the ones worth reading.
    //
    // Fed straight into the sink: this is about how the report formats a full
    // histogram, not about how one gets filled.
    {
        wl_enable(0);
        wl_enable(1);
        // bw fixed at 8 and bh sweeping 8..39 puts 10000 samples in every one
        // of the 32 aspect bins.
        for (int k = 0; k < WL_BINS; ++k)
            for (int j = 0; j < 10000; ++j)
                wl_note(0, 5, 8, 8 + k, 0, 0, WL_HAS_BOX);
        g_replies.clear();
        wiicam_cam_command("camlearn?");
        std::string wide;
        for (size_t i = 1; i < g_replies.size(); ++i)
            if (g_replies[i].compare(0, 22, "CAM: hist c=0 f=aspect") == 0)
                wide = g_replies[i];
        ck(!wide.empty(), "the widest histogram line is in the report");
        ck(hist_line_bins(wide) == WL_BINS,
           "and it still carries all 32 bins with five-digit counts in every "
           "one of them -- the widest line the report can ever emit must fit "
           "the buffer that formats it");
        const std::string tail = " 10000\n";
        ck(wide.size() > tail.size()
           && wide.compare(wide.size() - tail.size(), tail.size(), tail) == 0,
           "...right through to the LAST bin, which is where the clamped "
           "outliers are and the first thing a truncation loses");
    }

    // ---- camreset stops the capture too ------------------------------------
    // It is the command a user reaches for when nothing works, and leaving the
    // capture running past it would go on filling one distribution from two
    // configurations -- with everything else on the gun having just changed.
    {
        arm_clean();
        frame(RIG);
        ck(wl_enabled() == 1 && wl_frames() == 1u, "a capture is running");
        g_replies.clear();
        wiicam_cam_command("camreset");
        ck(wl_enabled() == 0, "camreset stops the capture with everything else");
        // camreset also puts the format back to basic, so drive the plain
        // entry point: the point is that nothing accumulates, whatever arrives.
        {
            int rpx[4] = {256, 768, 256, 768};
            int rpy[4] = {240, 240, 528, 528};
            float sx = 0.0f, sy = 0.0f;
            for (int i = 0; i < 4; ++i) {
                rpx[0] += (i & 1) ? 1 : -1;
                g_t += DT;
                wiicam_aim_process(rpx, rpy, 0xF, g_t, &sx, &sy);
            }
        }
        ck(wl_frames() == 1u,
           "...and no frame after it is counted, so the capture cannot span "
           "the reset that changed the rig underneath it");
    }

    printf("\nwiicam learn: %s (%d failures)\n", fails ? "FAILED" : "ALL PASS", fails);
    return fails ? 1 : 0;
}

// ---- two branches this test used to be unable to reach ---------------------
// Kept because the reasoning is still worth having, and because the reason it
// no longer applies is the whole shape of the negative class.
//
// 1. CLASS 1 WAS UNREACHABLE while the only label was "four real corners". The
//    wiicam has exactly four object slots and only KEPT blobs are handed to
//    quad_update, so a rejected blob leaves three points and r.n_real can be
//    at most three. r.n_real == 4 therefore implies every blob was kept, and
//    `wl_note(keep[i] ? 0 : 1, ...)` under that label could only ever pass 0:
//    the positive class filled while the negative one stayed structurally
//    empty, and the module would have looked like it was working.
//
//    The firmware took the second label the note asked for. A frame with three
//    real corners and a RECONSTRUCTED fourth, with exactly one blob rejected,
//    says where the missing LED must be -- so the rejected blob can be judged
//    on position alone, and one sitting further than twice the resolver's own
//    association radius from every corner was not an LED whatever it looked
//    like. That is still a verdict no gate voted in. "The NEGATIVE class, and
//    the second way into it" above drives it both ways round, through the size
//    window and through the shape gate, and the two produce the same sample.
//
// 2. THE ODD-COUNT MEDIAN came with it. `m` in the intensity median is the
//    number of KEPT blobs, which under a four-corner label was always 4 -- so
//    `(m & 1) ? v[m/2] : ...` could only take the even branch. A three-corner
//    frame with one blob rejected leaves three, and the odd branch is pinned
//    in that same block on intensities chosen so the middle value and the mean
//    of the middle pair land in different bins.
