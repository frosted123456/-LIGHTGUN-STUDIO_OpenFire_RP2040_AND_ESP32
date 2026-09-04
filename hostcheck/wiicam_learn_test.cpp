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
//
// The NEGATIVE label is that same property read from the other end, and it is
// the one that has already been got wrong once. The first rule asked whether a
// GATE had rejected the blob -- which on a gun with every gate at 0, the
// shipped default, can never be true. The negative class was structurally
// empty, 'camfit' could never reach a verdict on a gun straight out of the
// box, and the module looked like it was working while answering half the
// question it exists for. The rule now is purely positional: a blob further
// than TWICE the resolver's association radius from every resolved corner.
// "no gate switched on anywhere, and a sample is recorded anyway" is therefore
// not one assertion among many, it is the regression, and it is labelled as
// such where it appears.
//
// Two more blocks below leave the sink and reach into wiicam_aim.cpp's refusal
// floors, because both are read straight out of these histograms and both have
// already been wrong. The floor is the MAX of this capture and the stored
// 'fit0' record: reading the LIVE edge first let a thin capture LOWER the
// floor and accept a ceiling that blinds the gun at play distance. And the
// AREA histogram's top bin is a CLAMP bucket -- "at least 31" -- so on a rig
// whose blobs run past it the pixel floor answers "unbounded" rather than
// handing 31 back as a maximum, which accepted a ceiling of 32 on a rig whose
// LEDs are larger than that. Both are driven through real frames here, where
// "too thin" and "off the end of the scale" are properties of a CAPTURE
// rather than of a number handed to wl_note.
//
// And the LED HEIGHT edge is now the top of the HEAVIEST CONTIGUOUS RUN rather
// than the absolute highest occupied bin, because a daylight capture came back
// with LED heights 2..7 and then, twenty-three empty bins higher, 32 samples
// at 31 -- the sun, filed as a resolver-confirmed LED. A cold-start lock seeds
// on whichever four blobs it sees, so sun-plus-three-LEDs is as
// self-consistent as four LEDs and the 'locked' guard tested further down does
// NOT exclude those frames. The block that pins this feeds that capture
// literally; it also pins the SHORT capture that broke the first attempt at
// this (walking up from the mode, which started inside the contamination and
// reported it as the body), the tie rule, the flip point where the sun really
// does outweigh the LEDs, and the two sides that deliberately did NOT get any
// of this -- area, which has arithmetic holes, and the stray minimum, where an
// outlier can only refuse a gate rather than loosen one.
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include "wiicam_aim.h"
#include "wiicam_learn.h"
#include "aim_runtime.h"
#include "quad_resolver.h"
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
// One slot per KEY, not one slot shared by all of them. There are three u32
// keys now -- "gate0" the size window, "gate1" the shape gate, "fit0" the
// three measurements the shape ceiling was derived from -- and a key-blind
// slot lets any of them answer for the others. That is not a harmless stub
// shortcut here: the bhmax and pxmax floors take the MAX of the live histogram
// and aim_fit_load(), so a shape-gate word handed back as a provenance record
// invents a floor out of gate bits and refuses a setting this file asserts is
// taken -- and it does it in the direction that cannot be seen, since a floor
// only ever refuses.
struct U32Slot { char key[16]; uint32_t v; bool have; };
static U32Slot g_u32s[4];       // gate0 gate1 fit0, plus room
static U32Slot* u32_find(const char* k){
    for (auto& s : g_u32s) if (s.have && k && !strcmp(s.key, k)) return &s;
    return nullptr; }
// All three u32 keys by NAME. camreset erases the size gate's "gate0", the
// shape gate's "gate1" AND the provenance record's "fit0", and a stub that only
// knows the first lets the other erases fall through to the calibration blob
// -- wiping a setting the command never mentions while leaving the one it did.
static bool is_u32key(const char* k){
    return k && (!strcmp(k, "gate0") || !strcmp(k, "gate1")
                                     || !strcmp(k, "fit0")); }
esp_err_t nvs_erase_key(nvs_handle_t, const char* k){
    if (is_u32key(k)) { if (U32Slot* s = u32_find(k)) s->have = false; }
    else if (is_lens(k)) g_lhave = false; else g_bhave = false;
    return ESP_OK; }
esp_err_t nvs_set_i16(nvs_handle_t, const char*, int16_t){ return ESP_OK; }
esp_err_t nvs_get_i16(nvs_handle_t, const char*, int16_t*){ return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_set_u32(nvs_handle_t, const char* k, uint32_t v){
    if (U32Slot* s = u32_find(k)) { s->v = v; return ESP_OK; }
    for (auto& s : g_u32s)
        if (!s.have) { strncpy(s.key, k, 15); s.v = v; s.have = true; return ESP_OK; }
    return -1; }
esp_err_t nvs_get_u32(nvs_handle_t, const char* k, uint32_t* v){
    if (U32Slot* s = u32_find(k)) { *v = s->v; return ESP_OK; }
    return ESP_ERR_NVS_NOT_FOUND; }
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

    // ---- the envelope --------------------------------------------------
    // What the two distributions say at their EDGES, which is all a ceiling
    // needs: the tallest box a resolver-confirmed LED produced, the shortest
    // box a confirmed stray produced, and the two blob counts behind them.
    // Read out of the histograms rather than tracked beside them, so there is
    // one source of truth and a bin that moved cannot disagree with an edge
    // that did not.
    //
    // The empty answer is -1 and not 0, and that single distinction is why
    // this block exists. Zero is a MEASUREMENT -- "this rig's LEDs are 0 tall"
    // -- and a rig that has measured nothing would hand 'camfit' a gap of
    // stray_min - 0 and a ceiling drawn from a capture that never happened.
    // The bhmax floor reads the same field, so a 0 there also stops refusing
    // the settings that blind a gun, in exactly the state (fresh boot, empty
    // histograms) where the user is most likely to be typing numbers at it.
    {
        wl_envelope_t e;
        memset(&e, 0x5A, sizeof(e));      // poison: an untouched field shows up
        wl_enable(0); wl_enable(1);       // the off -> on edge clears
        wl_envelope(&e);
        ck(e.led_max_h == -1 && e.led_max_px == -1
           && e.stray_min_h == -1 && e.stray_min_px == -1,
           "an empty sink reports -1 on all four edges -- 'nothing measured', "
           "which is a different answer from 'measured zero' and the only one "
           "a gate may refuse to act on");
        ck(e.led_abs_max_h == -1,
           "...the absolute height max included: it is a second reading of the "
           "same histogram, so it has to agree about the empty case or the "
           "report and the floor disagree on a gun nobody has pointed at a bar");
        ck(e.led_outliers_h == 0ul,
           "...and the outlier COUNT is 0, which for a count is the honest "
           "empty value -- the poison above this would leave it enormous, and "
           "an enormous outlier count prints a contamination warning on a rig "
           "that has measured nothing at all");
        ck(e.led_n == 0u && e.stray_n == 0u,
           "...and both blob counts read zero, which for a COUNT is the honest "
           "empty value: it is the edges that have no zero to report");
        // One line in the firmware, on a path the report, camfit and the
        // bhmax floor all take.
        wl_envelope(0);
        ck(true,
           "a null 'out' returns instead of storing four ints through it -- "
           "this line is only reached if the guard held");

        // One sample, each class in turn.
        wl_enable(0); wl_enable(1);
        wl_note(0, 3, 4, 6, 100, 100, WL_HAS_BOX);       // 4x6 box, 24 px
        wl_envelope(&e);
        ck(e.led_max_h == 6 && e.led_max_px == 24,
           "one confirmed LED sets both LED edges from its own box: height 6 "
           "out of WL_BH and area 24 out of WL_AREA, not the width sitting "
           "between them");
        ck(e.stray_min_h == -1 && e.stray_min_px == -1,
           "...and leaves the stray edges at -1: one class filling must never "
           "make the other read as measured, or a rig that has seen no stray "
           "at all gets a gap and a verdict out of nowhere");
        ck(e.led_n == 1u && e.stray_n == 0u,
           "...with the counts following the classes they were filed under");

        wl_note(1, 7, 3, 9, 100, 100, WL_HAS_BOX);       // 3x9 box, 27 px
        wl_envelope(&e);
        ck(e.stray_min_h == 9 && e.stray_min_px == 27,
           "one confirmed stray sets both stray edges the same way");
        ck(e.led_max_h == 6 && e.led_max_px == 24,
           "...without disturbing the LED edges beside them");
        ck(e.led_n == 1u && e.stray_n == 1u,
           "...and each count is still its own class's");

        // MAX-ward on the LED side, MIN on the stray side, and the two are not
        // interchangeable. Both are the conservative direction: a ceiling
        // taken from the TALLEST LED can only ever be too loose, which costs
        // nothing but a gate that does less, while the SHORTEST stray is the
        // first thing any gate has to clear. Swapped, the pair still reads as
        // two plausible numbers -- 3 and 25 rather than 9 and 12 -- so both
        // classes get a spread on both sides of the other's edge.
        //
        // The LED heights are CONTIGUOUS here (3, 4, 5), and that is now part
        // of what the assertion means: the height edge is the top of the
        // heaviest contiguous run, so a distribution that is ONE run reads as
        // its own highest bin exactly as the absolute max used to. The gapped
        // case -- where the two readings disagree, and where the sun sits --
        // has a block of its own further down.
        wl_enable(0); wl_enable(1);
        wl_note(0, 3, 1,  3, 0, 0, WL_HAS_BOX);          // h 3,  area 3
        wl_note(0, 3, 2,  4, 0, 0, WL_HAS_BOX);          // h 4,  area 8
        wl_note(0, 3, 3,  5, 0, 0, WL_HAS_BOX);          // h 5,  area 15
        wl_note(1, 3, 2, 12, 0, 0, WL_HAS_BOX);          // h 12, area 24
        wl_note(1, 3, 1, 25, 0, 0, WL_HAS_BOX);          // h 25, area 25
        wl_note(1, 3, 2, 15, 0, 0, WL_HAS_BOX);          // h 15, area 30
        wl_envelope(&e);
        ck(e.led_max_h == 5 && e.led_max_px == 15,
           "the LED side reads MAX-ward -- heights 3, 4 and 5 read as 5, never "
           "as the first one seen and never as an average: a ceiling has to "
           "clear every LED this rig has produced");
        ck(e.led_abs_max_h == 5 && e.led_outliers_h == 0ul,
           "...and with no gap in the run the body edge and the absolute max "
           "are the same number, with nothing set aside: contiguity only ever "
           "changes the answer on a distribution that has a hole in it");
        ck(e.stray_min_h == 12 && e.stray_min_px == 24,
           "and the stray side is the LOWEST -- heights 12, 25 and 15 read as "
           "12, the first stray a gate would have to get under");
        ck(e.led_n == 3u && e.stray_n == 3u,
           "...over three blobs a side, which is also what wl_blobs reports");

        // Bin 31 is a CLAMP bucket. It means "at least this", not "exactly
        // 31", and reporting it as the edge is the honest reading for a
        // ceiling and the conservative one for a floor.
        wl_enable(0); wl_enable(1);
        wl_note(0, 3, 8, 200, 0, 0, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.led_max_h == WL_BINS - 1 && e.led_max_px == WL_BINS - 1,
           "a box far past the end of the array reports the clamp bucket -- "
           "31, 'at least 31', which is what the top bin is for");
        ck(e.led_abs_max_h == WL_BINS - 1 && e.led_outliers_h == 0ul,
           "...and when the clamp bucket is the only run there is, it is the "
           "body rather than an outlier above one: a rig whose every LED lands "
           "there is a rig with one bin of distribution, and nothing about it "
           "is contamination");
        ck(e.led_max_h != 200 % WL_BINS,
           "...and not 8, which is what a wrapping bin would have made of a "
           "200-row blob: an ordinary LED height, and a ceiling drawn UNDER "
           "every stray in the capture");
        wl_note(1, 3, 8, 200, 0, 0, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.stray_min_h == WL_BINS - 1,
           "a clamped stray reads 31 too: the floor says the strays start no "
           "lower than the top bin, which leaves camfit reporting no gap "
           "rather than inventing one out of a bin index");

        // 0xFFFF is where a bin stops counting. The envelope reads WHICH bins
        // are occupied and never how full they are, so a capture long enough
        // to saturate has to report the same edge -- and the bin must not wrap
        // to zero, which would delete the tallest LED the rig ever produced
        // from the very ceiling drawn to clear it.
        wl_enable(0); wl_enable(1);
        for (long i = 0; i < 70000; ++i) wl_note(0, 3, 2, 7, 0, 0, WL_HAS_BOX);
        wl_envelope(&e);
        ck(hat(0, WL_BH, 7) == 0xFFFFu,
           "70,000 identical blobs saturate their bin at 0xFFFF instead of "
           "wrapping through zero");
        ck(e.led_max_h == 7 && e.led_max_px == 14,
           "...and the envelope still reports that bin as the edge: a wrapped "
           "bin reads as EMPTY, so the answer would silently become the next "
           "occupied bin down, or -1 if there is none");
        ck(e.led_n == 70000u,
           "...while the blob COUNT keeps climbing past 65535, because camfit "
           "gates on it at 500 and a count that saturated with its bins would "
           "stall a rig that had measured plenty");

        // Size-only samples, as an extended-format capture produces. They are
        // real blobs and they count as blobs, but they carry no box -- so the
        // envelope has to go on saying -1. This is the pair camfit tests
        // SEPARATELY for exactly this reason: led_n alone says there is plenty
        // of data, and the ceiling would be drawn from a height nobody
        // measured.
        wl_enable(0); wl_enable(1);
        for (int i = 0; i < 600; ++i) wl_note(0, 2, 0, 0, 0, 0, 0);
        wl_envelope(&e);
        ck(e.led_n == 600u, "600 extended-mode blobs are 600 blobs");
        ck(e.led_max_h == -1 && e.led_max_px == -1,
           "...and still no envelope at all: a capture carrying no box must "
           "not read as one that measured zero, which is a ceiling of zero and "
           "a gun that sees nothing");
        wl_enable(0);
        wl_reset();
    }

    // ======================================================================
    // THE SUN IN THE LED CLASS: the height edge is the top of the BODY
    // ======================================================================
    // The LED height edge is the top of the HEAVIEST CONTIGUOUS RUN -- the
    // connected group of bins holding the most samples -- with everything
    // above it counted as outliers. That is not tidiness; it is the only
    // signature this sink has for a specific, measured contamination.
    //
    // MASS, not the tallest bin's run. The first cut of this walked up from
    // the MODE, and that inverted on a short capture: LEDs at 2..7 with ten
    // samples a bin against thirty-two at 31 makes bin 31 the mode, so the
    // walk STARTED in the contamination, reported 31 as the body, counted zero
    // outliers and printed no warning at all -- strictly worse than the
    // absolute max it replaced, because it claimed to have handled the very
    // thing it had just let through. Sixty samples of LED against thirty-two
    // of sun is still mostly LED, and mass is what says so. That case is the
    // first assertion in this block, by itself, because it is the regression.
    //
    // A daylight capture with no gate at all came back with LED heights 2..7
    // and then, after twenty-three EMPTY bins, 32 samples at 31. That is the
    // sun, filed as a resolver-confirmed LED. The resolver's lock is
    // self-consistency, not geometry: at a cold start it seeds on whichever
    // four blobs it sees and then holds that shape, so sun-plus-three-LEDs
    // locks exactly like four LEDs until the player moves and the parallax
    // breaks it. Those frames are honestly 'locked && n_real == 4' -- the
    // guard tested further down does NOT exclude them, and there is no flag
    // that could. What the sun cannot do is CONNECT to the LED body: a point
    // source and a window are a dozen bins apart, and the gap is the whole
    // evidence.
    //
    // It matters because this edge is the refusal FLOOR under bhmax. Read as
    // the absolute max, this capture puts the floor at 31 and refuses the
    // bhmax of 8 that was measured to catch 84% of strays at zero LED cost --
    // the sun learned as an LED becomes the thing that refuses the cut that
    // would have removed it. Read as the body, the floor is 7 and 8 lands.
    {
        wl_envelope_t e;
        // ---- the capture, literally -----------------------------------------
        // Heights and sample counts exactly as they came off the gun. Not
        // scaled: 11,991 samples is nowhere near the 65,535 at which a bin
        // stops counting, so the shape asserted here is the shape measured.
        static const struct { int h; int n; } CAP[] = {
            {  2, 3219 }, {  3, 5938 }, {  4, 2551 },
            {  5,  221 }, {  6,   21 }, {  7,    9 },
            { 31,   32 },            // ...and the sun, 23 empty bins above
        };
        static const int NCAP = 7;
        wl_enable(0); wl_enable(1);
        for (int k = 0; k < NCAP; ++k)
            for (int i = 0; i < CAP[k].n; ++i)
                wl_note(0, 2, 3, CAP[k].h, 100, 100, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.led_max_h == 7,
           "THE REAL CAPTURE: the LED height edge is 7, the top of the body "
           "that runs 2..7 -- and 7 is the number a ceiling has to clear, "
           "because it is the tallest height a blob connected to this rig's "
           "own distribution has ever produced");
        ck(e.led_abs_max_h == 31,
           "...while the ABSOLUTE highest bin is still reported as 31, "
           "separately: the contamination is not deleted, it is set aside, and "
           "a user who cannot see it cannot go and block the window");
        ck(e.led_outliers_h == 32ul,
           "...and the 32 samples above the body are counted, so the report "
           "can say how much of the capture was thrown away rather than "
           "quietly presenting a clean 7");
        ck(e.led_n == 11991ul,
           "...with the blob COUNT still the whole capture, all 11,991 of "
           "them: the outliers are excluded from the EDGE, not from the "
           "evidence that enough data was collected -- camfit gates on this "
           "count at 500 and must not be starved by a discount");

        // ---- THE INVERSION: a SHORT capture, where mode-based walking broke -
        // The same shape as the real capture and nothing like its margin: six
        // LED bins with TEN samples each, sixty in total, against the sun's
        // thirty-two in one bin. Bin 31 is now the tallest single bin in the
        // whole histogram.
        //
        // Mode-based, the walk started there: 31 came back as the body, the
        // outlier count came back 0, and no warning was printed -- so the
        // floor was the sun again and the report said the capture was clean.
        // That is strictly worse than the absolute max this replaced, because
        // the absolute max at least never claimed to have handled it.
        //
        // Sixty samples of LED against thirty-two of sun is still mostly LED,
        // and MASS is what says so: the run 2..7 weighs 60, the run at 31
        // weighs 32, and the heavier run is the body.
        wl_enable(0); wl_enable(1);
        for (int h = 2; h <= 7; ++h)
            for (int i = 0; i < 10; ++i)
                wl_note(0, 2, 3, h, 100, 100, WL_HAS_BOX);
        for (int i = 0; i < 32; ++i) wl_note(0, 2, 1, 31, 100, 100, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.led_max_h == 7,
           "TEN samples a bin against the sun's thirty-two in one: the edge is "
           "still 7. This is the inversion. The tallest single BIN is 31, so a "
           "mode-based walk started inside the contamination and handed the "
           "floor straight back to it -- the heaviest RUN is the six bins that "
           "weigh 60 between them, and mass is the only reading that survives "
           "a short capture");
        ck(e.led_outliers_h == 32ul,
           "...and the 32 sun samples are still COUNTED as outliers, which "
           "mode-based walking reported as 0 -- the warning went silent in "
           "exactly the case it was needed, so a user got a clean-looking 31 "
           "and no reason to doubt it");
        ck(e.led_abs_max_h == 31,
           "...with 31 still reported as the absolute max, so the report says "
           "the same thing about this capture as about the long one");

        // ---- the FLIP POINT, and it is honest ------------------------------
        // Five samples a bin, thirty in total, against the same thirty-two of
        // sun. The capture now holds more sun than LED, and the answer flips:
        // the run at 31 IS the heaviest run, so it is the body and there are
        // no outliers above it.
        //
        // That is the right answer rather than a hole. Nothing derived from a
        // capture that is mostly stray light could be trusted, and the sink's
        // job is to report what it measured, not to guess which half was real.
        // What stands between this and a gate is camfit's 500-sample gate,
        // which is asserted on this exact histogram in wiicam_adapter_test.
        wl_enable(0); wl_enable(1);
        for (int h = 2; h <= 7; ++h)
            for (int i = 0; i < 5; ++i)
                wl_note(0, 2, 3, h, 100, 100, WL_HAS_BOX);
        for (int i = 0; i < 32; ++i) wl_note(0, 2, 1, 31, 100, 100, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.led_max_h == 31 && e.led_outliers_h == 0ul,
           "thirty LED samples against thirty-two of sun and the sun IS the "
           "body: 31, with nothing above it to set aside. The mechanism flips "
           "only when the capture holds more contamination than signal, which "
           "is a capture no gate should be derived from at all -- and saying "
           "so plainly beats inventing a body out of the minority");
        ck(e.led_n == 62ul,
           "...on 62 blobs, which is what makes it safe: camfit refuses under "
           "500, so the flip is unreachable by anything that could produce a "
           "verdict");

        // ---- the TIE keeps the LOWER run -----------------------------------
        // Thirty-two LED samples spread across 2..7 against thirty-two of sun
        // in one bin: exactly equal mass. Strictly-heavier-wins means the
        // first run encountered keeps it, which is the lower one.
        //
        // The safe direction, on both counts: a floor built on the lower run
        // refuses LESS, and a ceiling built on it is TIGHTER. And a capture
        // where LEDs and sun weigh the same is one nobody should derive from,
        // so the tie-break only has to avoid being actively harmful.
        wl_enable(0); wl_enable(1);
        {
            static const int TIE[6] = { 6, 6, 5, 5, 5, 5 };   // 32 across 2..7
            for (int k = 0; k < 6; ++k)
                for (int i = 0; i < TIE[k]; ++i)
                    wl_note(0, 2, 3, 2 + k, 100, 100, WL_HAS_BOX);
        }
        for (int i = 0; i < 32; ++i) wl_note(0, 2, 1, 31, 100, 100, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.led_max_h == 7 && e.led_outliers_h == 32ul,
           "equal mass -- 32 against 32 -- keeps the LOWER run: the edge is 7 "
           "and the sun is the outlier group, because a floor on the lower run "
           "refuses less and a ceiling on it is tighter. A >= comparison here "
           "would hand the tie to whichever run came last, which is the sun");

        // ---- two LED-side runs, no contamination at all --------------------
        // The heavy run wins on mass and a lighter REAL tail above it is set
        // aside. That is the same deliberate trade as before, now stated in
        // terms of mass rather than position: a real sparse tail costs the
        // floor a row or two, and the direction is chosen knowingly, because
        // too low a floor accepts a slightly tighter gate while too high a
        // floor refuses the only working one and the user cannot argue with it.
        wl_enable(0); wl_enable(1);
        for (int h = 2; h <= 5; ++h)
            for (int i = 0; i < 100; ++i)
                wl_note(0, 2, 3, h, 100, 100, WL_HAS_BOX);
        for (int h = 7; h <= 8; ++h)
            for (int i = 0; i < 7; ++i)
                wl_note(0, 2, 3, h, 100, 100, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.led_max_h == 5,
           "a heavy run at 2..5 and a light real tail at 7..8, separated by an "
           "empty bin 6: the heavy run wins on MASS and the edge is 5 -- "
           "contamination is not the only thing this discounts, and pretending "
           "otherwise would make the trade invisible");
        ck(e.led_abs_max_h == 8 && e.led_outliers_h == 14ul,
           "...with the tail's own top reported as 8 and all 14 of its samples "
           "counted: a real tail set aside looks exactly like the sun in the "
           "report, which is correct -- the sink cannot tell them apart and "
           "must not pretend to");

        // ---- a non-empty bin 0 does not truncate anything -------------------
        // The sensitivity-2 captures have a non-empty bin 0 (a box whose 7-bit
        // corners came out equal), bin 1 empty, and then the body. Bin 0 is a
        // run of its own, weighing 40 against the body's 1800, so the body
        // wins and bin 0 is simply not the answer.
        //
        // Worth its own case because the obvious cheap implementations both
        // get it wrong: "walk up from the first occupied bin" stops at 0 and
        // reports a ceiling of ZERO -- a gate that rejects every blob on the
        // rig -- and "the first run" does the same.
        wl_enable(0); wl_enable(1);
        for (int i = 0; i < 40; ++i) wl_note(0, 2, 3, 0, 100, 100, WL_HAS_BOX);
        for (int h = 2; h <= 7; ++h)
            for (int i = 0; i < 300; ++i)
                wl_note(0, 2, 3, h, 100, 100, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.led_max_h == 7,
           "a non-empty bin 0 with bin 1 empty below the body still gives 7: "
           "the body is the heaviest run, so a small tally at the very bottom "
           "cannot truncate it to nothing");
        ck(e.led_outliers_h == 0ul,
           "...and nothing is counted as an outlier, because bin 0 is BELOW "
           "the chosen run: only samples above it are contamination, and a "
           "floor is not made safer by discounting the small end");

        // ---- ONE empty bin is enough to separate two runs -------------------
        // Which side wins is a question of mass, above. This is the other
        // half: how much of a gap it takes to make two runs at all, and the
        // answer is a SINGLE empty bin. That is deliberately the least
        // separation there can be, because it is all the separation the sink
        // is guaranteed -- the sun happened to sit twenty-three bins clear,
        // and nothing says the next contaminant will.
        //
        // The cost is a genuine sparse tail sample above a hole being set
        // aside, which puts the floor a row or two under the tallest real LED.
        // The direction is chosen knowingly: too low a floor accepts a
        // slightly tighter gate, while too high a floor refuses the only
        // working one and the user has no way to argue with it.
        wl_enable(0); wl_enable(1);
        for (int h = 2; h <= 5; ++h)
            for (int i = 0; i < 100; ++i)
                wl_note(0, 2, 3, h, 100, 100, WL_HAS_BOX);
        for (int i = 0; i < 3; ++i) wl_note(0, 2, 3, 7, 100, 100, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.led_max_h == 5,
           "heights 2..5 and three samples at 7 with ONLY bin 6 empty between "
           "them are two runs, not one: the heavy run ends at 5, so a single "
           "empty bin is all it takes to separate a body from what sits above "
           "it");
        ck(e.led_abs_max_h == 7 && e.led_outliers_h == 3ul,
           "...with 7 reported as the absolute max and the three samples "
           "counted, so the trade is visible in the report rather than silent");

        // ---- AREA is deliberately NOT contiguous ----------------------------
        // Area is a PRODUCT, w*h, so its histogram has arithmetic holes: no
        // blob has an area of 13, or 5 on a rig whose boxes are 2 and 3 wide.
        // A contiguity walk therefore stops at the first gap in the number
        // line rather than at a gap in the DATA, and on the sensitivity-1
        // capture -- areas 1..16 with nothing at 5 -- it would report 4 with
        // real LEDs at 16. That is a floor far below the rig, which is the
        // direction that accepts a blinding ceiling. So this side stays the
        // absolute max, and the sun contaminating it is accepted: an inflated
        // area edge only ever REFUSES a pxmax, and a refusal is arguable.
        wl_enable(0); wl_enable(1);
        wl_note(0, 2, 2, 2, 100, 100, WL_HAS_BOX);      // area 4
        wl_note(0, 2, 2, 3, 100, 100, WL_HAS_BOX);      // area 6
        wl_note(0, 2, 3, 3, 100, 100, WL_HAS_BOX);      // area 9
        wl_note(0, 2, 4, 4, 100, 100, WL_HAS_BOX);      // area 16
        wl_envelope(&e);
        ck(e.led_max_px == 16,
           "areas 4, 6, 9 and 16 -- with nothing at 5 -- read as 16, the "
           "absolute max: a heaviest-run walk would see four runs of one "
           "sample each, keep the lowest on the tie, and report 4 -- a ceiling "
           "under every real LED on the rig");
        ck(e.led_max_h == 4,
           "...while the HEIGHTS in the same capture (2, 3, 3, 4) are "
           "contiguous and read as 4, so the two sides are being read by "
           "different rules on one histogram set and each is asserted where it "
           "would be wrong to swap them");

        // ---- the STRAY minimum is deliberately NOT contiguous ---------------
        // An outlier on the stray side can only ever SHRINK the gap between
        // the two distributions, and a shrunken gap refuses a gate rather than
        // loosening one. NO SAFE GATE is a survivable answer; a ceiling that
        // cuts real corners is not. So a single low stray counts, even though
        // it is exactly the shape that gets discounted on the LED side.
        wl_enable(0); wl_enable(1);
        wl_note(1, 7, 3, 3, 100, 100, WL_HAS_BOX);      // ONE stray at 3
        for (int h = 9; h <= 10; ++h)
            for (int i = 0; i < 50; ++i)
                wl_note(1, 7, 3, h, 100, 100, WL_HAS_BOX);
        wl_envelope(&e);
        ck(e.stray_min_h == 3,
           "one stray at 3 against a body of fifty each at 9 and 10 reads as "
           "3, not 9: the stray floor is an absolute MIN and stays one, "
           "because the asymmetry of the two mistakes is the whole argument "
           "-- a low stray costs a gate, a high LED costs the corners");
        wl_enable(0);
        wl_reset();
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

    // ======================================================================
    // THE COLD-START HOLE: n_real == 4 is not a verdict -- 'locked' is.
    // ======================================================================
    // wiicam_learn.h promises that a positive sample is "four blobs forming a
    // plausible rectangle at a plausible scale". Nothing in n_real checks
    // either half of that. n_real is how many of the four published corners
    // were actually SEEN this frame, and it reads 4 on two paths that have
    // verified nothing whatsoever:
    //
    //   * the cold-start raw passthrough, which republishes whatever four
    //     blobs it was handed, and
    //   * the frames immediately after the angular seed, which sorts four
    //     blobs by angle around their own centroid and asks nothing else of
    //     them -- not a rectangle, not a scale, not a residual.
    //
    // The verification lives entirely in 'locked': it comes true only after
    // quad_default_config().lock_frames consecutive all-four-real frames, and
    // that wait IS the rectangle check the header is claiming. So the hook in
    // wiicam_aim.cpp tests 'r.locked && r.n_real == 4'. It once tested n_real
    // alone, and on the frames before the lock it learned a window and a lamp
    // as LEDs.
    //
    // This is NOT a boot-time-only hole, which is what makes it a regression
    // test rather than a footnote. Every quad_reset(0) reopens it, and three
    // '~cam' keys the tools send routinely call quad_reset(0): 'res:',
    // 'mirx:' and 'miry:'. A user flipping the mirror to get their axes the
    // right way round, hours into a session, with a capture armed, walks
    // straight back into the window -- which is why the two halves below are
    // driven once from a real wiicam_aim_begin() and once from a mid-session
    // 'mirx:'.
    //
    // Both halves matter and they pull in opposite directions. A condition
    // that never comes true would starve the very class it was added to
    // protect, and an empty positive histogram is indistinguishable from a rig
    // nobody ever pointed at the bar -- so the genuine rectangle is checked
    // first, and the wait is MEASURED rather than compared against a constant.
    //
    // PRELOCK is how wide the hole is, and it is derived from the resolver's
    // own tuning rather than written down: the lock wants lock_frames
    // consecutive all-four-real frames, seed() banks the first of them before
    // the first frame's own increment, so the lock lands ON frame
    // lock_frames - 1 and the frames BEFORE it number lock_frames - 2.
    //
    // Deliberately NOT derived from the measured wait below. Under the bug the
    // positive class records from frame one, so a junk run sized from what the
    // sink was seen to do would shrink to nothing in exactly the build it has
    // to catch, and pass by feeding no frames at all. The two numbers are kept
    // independent and then checked against each other.
    const int PRELOCK = quad_default_config().lock_frames - 2;
    int LOCK_N = 0;
    {
        // ---- the lock does arrive, and this is how many frames it takes ----
        // LOCK_N is the answer, measured, and printed into the assertion
        // message on purpose. A future change to lock_frames -- or to seed(),
        // whose pre-banked frame is why the lock lands one frame earlier than
        // lock_frames alone would suggest -- shows up HERE as that number
        // moving, with both numbers named, rather than somewhere else as a
        // capture that mysteriously recorded nothing.
        wiicam_cam_command("cam=fmt:2,bmin:0,bmax:15,rtol:0,"
                           "bhmax:0,pxmax:0,armax:0");
        wiicam_cam_command("cam=mirx:1");     // quad_reset(0): a cold resolver
        arm_clean();
        const unsigned long r4a = blobstat("br4=");
        for (int i = 1; i <= 16 && !LOCK_N; ++i) {
            frame(RIG);
            if (wl_frames()) LOCK_N = i;
        }
        static char msg_lock[512];
        snprintf(msg_lock, sizeof msg_lock,
                 "a genuine rectangle held steady STILL fills the positive "
                 "class, and the first frame it fills from is the first LOCKED "
                 "one: %d frames of it were needed, against the %d the "
                 "resolver's own lock_frames=%d predicts. The fix must narrow "
                 "this class, never starve it -- and a 1 here is the bug, "
                 "learning from the angular seed before anything was verified",
                 LOCK_N, PRELOCK + 1, quad_default_config().lock_frames);
        ck(LOCK_N == PRELOCK + 1 && wl_blobs(0) > 0u, msg_lock);
        ck(blobstat("br4=") == r4a + (unsigned long)LOCK_N,
           "...and every single one of those frames was ALREADY four-real: "
           "what the class was waiting for was the lock, not the corners, "
           "which is exactly why n_real on its own could never have been the "
           "test");
        ck(wl_frames() == 1u && wl_blobs(0) == 4u,
           "...so exactly one frame and its four blobs are in, with no "
           "backlog: the frames watched before the lock are not admitted "
           "retrospectively once it arrives");
        frame(RIG);
        frame(RIG);
        ck(wl_frames() == 3u && wl_blobs(0) == 12u,
           "...and it goes on filling for as long as the lock holds -- four "
           "blobs per confirmed frame, which is what the whole capture is for");
    }

    // The repro, exactly as reported: four blobs that are nothing like a
    // rectangle. Raw (100,100) (140,108) (180,700) (900,120) -- with mirx on,
    // that is (216,23) (207,25) (198,160) (29,28) in the resolver's 240x176
    // space: three bunched down the right-hand edge, one of them 135 rows
    // below the other two, and a fourth stranded on the far left. No pairing
    // of them is a rectangle at any scale. Two LEDs of a bar, a lamp and a
    // window, which is the arrangement a user actually has in the room.
    //
    // The boxes are the point of the third case below: the third blob is 3
    // wide and 9 tall, so if this junk reaches the positive class the tallest
    // "LED" this rig has ever measured becomes 9 rows.
    static const FullObj JUNK[4] = {
    //    x    y  sz  xmn ymn xmx ymx  inten
        { 100, 100, 2,  10, 20, 14, 24,  60 },   // 4x4
        { 140, 108, 3,  10, 20, 16, 26, 100 },   // 6x6
        { 180, 700, 4,  10, 20, 13, 29, 140 },   // 3x9 -- the tall one
        { 900, 120, 5,  10, 20, 15, 27, 190 },   // 5x7
    };
    {
        // ---- THE REGRESSION: the first frames after wiicam_aim_begin() -----
        // begin() opens with quad_reset(0), so these are a cold resolver's
        // first frames in the most literal sense. PRELOCK of them is the whole
        // width of the window: every frame before the lock, and not one after
        // it. (Past the lock the resolver has watched these four points hold
        // still for lock_frames and has decided they ARE a rigid rig -- at
        // which point learning from them is the resolver's judgement rather
        // than the absence of one, and a different argument entirely.)
        ck(PRELOCK >= 1,
           "there is a window to test at all: lock_frames leaves at least one "
           "four-real frame standing before the lock, which is the frame the "
           "bug learned from");
        wiicam_aim_begin();
        wiicam_cam_command("cam=fmt:2,bmin:0,bmax:15,rtol:0,"
                           "bhmax:0,pxmax:0,armax:0");
        arm_clean();
        const unsigned long r4a   = blobstat("br4=");
        const unsigned long cold_a= blobstat("bcold=");
        const unsigned long rej_a = blobstat("brej=");
        const unsigned long srej_a= blobstat("bsrej=");
        for (int i = 0; i < PRELOCK; ++i) frame(JUNK);

        ck(blobstat("br4=") == r4a
           && blobstat("bcold=") == cold_a + (unsigned long)PRELOCK,
           "every one of these frames DID reach the resolver -- it is the seed "
           "veto that turns them away, not a gate upstream -- and the resolver "
           "hands them straight back as a COLD PASSTHROUGH: bcold counts all "
           "of them and br4 none, because four blobs in a shape no rig could "
           "produce are not four corners");
        ck(blobstat("brej=") == rej_a && blobstat("bsrej=") == srej_a,
           "...and not one gate rejected anything: all four junk blobs were "
           "kept and offered, with the size window wide open and every shape "
           "knob at 0, exactly as a gun ships");
        ck(wl_blobs(0) == 0u && wl_frames() == 0u,
           "AND NOT ONE POSITIVE SAMPLE IS RECORDED. This is the regression. "
           "Four blobs in a shape no rig could produce, on a resolver that has "
           "verified nothing yet, are not LEDs -- and n_real alone said they "
           "were, which is how a lamp and a window got into the distribution "
           "that decides what an LED looks like");
        {
            wl_envelope_t e;
            wl_envelope(&e);
            ck(e.led_max_h == -1 && e.led_max_px == -1 && e.led_n == 0ul,
               "...so the envelope still reads -1, 'this rig has never been "
               "measured'. That is a different answer from a measurement of "
               "zero, and it is the only honest one after a capture that saw "
               "no confirmed frame");
        }
    }
    {
        // ---- and the window reopens LONG AFTER BOOT ------------------------
        // 'res:', 'mirx:' and 'miry:' each end in quad_reset(0), and the tools
        // send all three -- 'mirx:' and 'miry:' every time someone squares up
        // their axes, 'res:' on every entry to and exit from a lens sweep.
        // Each one drops the model, the lock and the four live slots, so the
        // next frame is a cold start with a capture still armed. A hole only
        // open in the first milliseconds after power-up would be hard to hit;
        // this one is a keypress away at any point in a session.
        wiicam_cam_command("cam=mirx:1");
        arm_clean();
        const unsigned long r4b = blobstat("br4=");
        const unsigned long cold_b = blobstat("bcold=");
        for (int i = 0; i < PRELOCK; ++i) frame(JUNK);
        ck(blobstat("br4=") == r4b
           && blobstat("bcold=") == cold_b + (unsigned long)PRELOCK,
           "the same junk is a cold passthrough again after a mid-session "
           "'mirx:' -- the reset really did take the lock down and hand the "
           "resolver back to a cold start, where the seed veto declines the "
           "set once more and bcold, not br4, is what moves");
        ck(wl_blobs(0) == 0u && wl_frames() == 0u,
           "...and the positive class stays empty there too: the guard is on "
           "the resolver's verdict, not on how long the gun has been running, "
           "so it holds on the hundredth quad_reset as well as the first");

        // ---- the knock-on, which is where the user actually feels it -------
        // A ceiling learned too loose would only mean the gate does nothing.
        // This is worse, and it is the harm worth pinning: rig_led_max_h()
        // reads the top occupied bin of class 0's HEIGHT histogram, and that
        // same number is the FLOOR under 'bhmax'. Junk in the positive class
        // therefore does not loosen a ceiling, it raises a floor -- the lamp
        // that got learned as an LED becomes the thing that refuses the cut
        // which would have removed it, and the refusal quotes a measurement
        // that never happened. The tall junk blob is 9 rows, so before the fix
        // 9 was the tightest bhmax this gun would accept and 'bhmax:4' came
        // back "below the tallest LED this rig has been measured at (9)".
        wiicam_cam_command("cam=bhmax:0");
        g_replies.clear();
        wiicam_cam_command("cam=bhmax:4");
        ck(g_replies.size() == 1
           && g_replies[0].find("not set") == std::string::npos
           && g_replies[0].find("bhmax=4 ") != std::string::npos,
           "a tight bhmax:4 is ACCEPTED after those junk frames -- before the "
           "fix it was refused, citing a 9-row 'LED' that was a lamp, and the "
           "user was locked out of the one setting that would have cut the "
           "lamp out");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=4 ") != std::string::npos,
           "...and it is really set, not merely un-refused: the floor that "
           "junk in the positive class puts under this knob is the whole "
           "user-visible cost of the bug");

        // Put the gun and the resolver back the way the blocks below expect
        // them: gates open, capture off, and a cold resolver rather than one
        // holding a half-built model of a lamp and a window.
        wiicam_cam_command("cam=bhmax:0,pxmax:0,armax:0,bmin:0,bmax:15,rtol:0");
        wiicam_cam_command("camlearn=on:0");
        wiicam_cam_command("cam=mirx:1");
    }

    // ---- the same floor, read from the OTHER side: a THIN live capture -----
    // The block above is about junk in the positive class RAISING the floor.
    // This one is its mirror, and it is a separate audited defect: an honest
    // but THIN capture LOWERING it.
    //
    // rig_led_max_h() and rig_led_max_px() take the MAX of this live histogram
    // and the 'fit0' record. They used to prefer the LIVE edge whenever the
    // histograms held anything at all, and the shape of that bug is why it is
    // worth a block of its own: a dim room, four small blobs and one confirmed
    // frame report a SMALL maximum, so live-first handed back a SMALL floor,
    // which accepted a ceiling that cuts the gun's own LEDs out at play
    // distance -- while a real, larger measurement of the same bar sat in
    // flash unused. And it did it silently, because a floor that stops
    // refusing produces no output at all.
    //
    // Note the asymmetry that made this easy to miss, because it is what a
    // reviewer's eye slides off: 'camfit' REFUSES to derive a ceiling from
    // fewer than 500 blobs, and this floor believed a single one. One capture
    // is not enough evidence to draw a ceiling from, and it was enough to let
    // one through.
    //
    // Driven through real frames rather than wl_note, because the point is a
    // capture too thin to be trusted producing a histogram the floor was
    // reading anyway -- and "too thin" is a property of the CAPTURE, not of a
    // number handed straight to the sink.
    {
        // The same rectangle the resolver already trusts -- only the boxes
        // change, and the resolver never sees a box. Four 3x2 LEDs: 2 rows
        // tall, 6 px of area, which is about what a bar looks like across a
        // dark room at the far end of the play range.
        static const FullObj THIN[4] = {
        //    x    y  sz  xmn ymn xmx ymx  inten
            { 256, 240, 1,  10, 20, 13, 22,  60 },
            { 768, 240, 1,  10, 20, 13, 22,  60 },
            { 256, 528, 1,  10, 20, 13, 22,  60 },
            { 768, 528, 1,  10, 20, 13, 22,  60 },
        };
        wiicam_cam_command("cam=mirx:1");        // quad_reset(0): cold resolver
        arm_clean();
        // An earlier capture of this rig, on record: 9 rows and 40 px. This is
        // what 'camsave' and 'camfit=apply' write, and it is the measurement
        // the live-first floor threw away.
        // 28 px rather than 40: at or above WL_BINS - 1 the pixel floor
        // answers UNBOUNDED instead of a number, which is a different refusal
        // and is driven on its own below.
        aim_fit_store(9, 12, 28);
        for (int i = 0; i < LOCK_N + 2; ++i) frame(THIN);
        {
            wl_envelope_t e;
            wl_envelope(&e);
            ck(e.led_max_h == 2 && e.led_max_px == 6 && e.led_n > 0ul,
               "the thin capture really did land -- 2-row, 6-pixel LEDs, "
               "resolver-confirmed -- so the histogram a live-first floor "
               "would have read is genuinely there and genuinely small");
            ck(e.led_n < 500ul,
               "...and on FEWER than 500 blobs, which is the count 'camfit' "
               "refuses to derive a ceiling from: the two bounds have to agree "
               "about how much evidence a number needs, and this is the "
               "arithmetic that showed they did not");
        }
        wiicam_cam_command("cam=bhmax:0,pxmax:0");
        g_replies.clear();
        wiicam_cam_command("cam=bhmax:3");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax 3 is below the tallest LED this rig has "
                                "been measured at (9)") != std::string::npos
           && g_replies[0].find("not set") != std::string::npos,
           "AND bhmax:3 IS REFUSED, NAMING THE STORED 9. This is the "
           "regression: live-first read the thin capture's 2, dropped the "
           "floor to 2, and took a ceiling of 3 without a word -- a gate one "
           "row over LEDs that measure 9 on this same bar in a lit room, so "
           "the gun goes blind the moment the light comes back");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=0 ") != std::string::npos,
           "...and nothing was written: a refusal that half-applied would "
           "leave the blinding ceiling in place with a warning beside it");
        g_replies.clear();
        wiicam_cam_command("cam=pxmax:20");
        ck(!g_replies.empty()
           && g_replies[0].find("pxmax 20 is below the largest LED this rig "
                                "has been measured at (28 px)") != std::string::npos
           && g_replies[0].find("not set") != std::string::npos,
           "and the area floor the same way, against the stored 28 px rather "
           "than the live 6 -- which is the whole reason 'fit0' grew a third "
           "field: without a stored pixel edge, a thin capture is the ONLY "
           "thing that floor can consult");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("pxmax=0 ") != std::string::npos,
           "...and it too stays where it was");
        // The live capture still WINS when it is the larger of the two, which
        // is the half that keeps a recapture meaningful: this rig has really
        // measured 2 rows just now, and 9 is still the bound because 9 is the
        // most this rig has ever been seen to produce.
        g_replies.clear();
        wiicam_cam_command("cam=bhmax:9");
        ck(g_replies.size() == 1
           && g_replies[0].find("not set") == std::string::npos
           && g_replies[0].find("bhmax=9 ") != std::string::npos,
           "the stored 9 itself is the first legal value: the floor is a MAX, "
           "so it is exactly as tight as the largest measurement this gun "
           "holds and never tighter");
        // Leave the store and the gun the way the blocks below found them.
        aim_fit_clear();
        wiicam_cam_command("cam=bhmax:0,pxmax:0,armax:0,bmin:0,bmax:15,rtol:0");
        wiicam_cam_command("camlearn=on:0");
        wiicam_cam_command("camlearn=reset");
        wiicam_cam_command("cam=mirx:1");
    }

    // ---- the capture that runs off the end of the AREA scale ---------------
    // WL_AREA's top bin is a CLAMP bucket: it means "at least 31", not
    // "exactly 31". The 'pxmax' floor is measured from it, and pxmax accepts
    // 0..63 -- so on a rig whose LED blobs are genuinely larger than 31 px the
    // measurement pins at the bucket and every value above it looks
    // unmeasured. Handed back as a maximum, that ACCEPTED pxmax:32 on a rig
    // whose LEDs are 40 px: a ceiling that cuts real corners out of every
    // frame, taken without a word. The floor answers "unbounded" instead.
    //
    // Driven through real frames because the saturation is a property of what
    // the SINK does with a real capture, not of a number handed to it: the
    // bucket is where wl_note's clamp puts a large blob, and this block is the
    // only place in either file where the clamp arrives that way.
    {
        // 8 wide by 4 tall: 32 px of area, one past the last bin there is, on
        // the same rectangle the resolver already trusts.
        static const FullObj BIG[4] = {
        //    x    y  sz  xmn ymn xmx ymx  inten
            { 256, 240, 1,  10, 20, 18, 24,  60 },
            { 768, 240, 1,  10, 20, 18, 24,  60 },
            { 256, 528, 1,  10, 20, 18, 24,  60 },
            { 768, 528, 1,  10, 20, 18, 24,  60 },
        };
        wiicam_cam_command("cam=mirx:1");        // quad_reset(0): cold resolver
        arm_clean();
        for (int i = 0; i < LOCK_N + 2; ++i) frame(BIG);
        {
            wl_envelope_t e;
            wl_envelope(&e);
            ck(e.led_max_px == WL_BINS - 1 && e.led_n > 0ul,
               "a resolver-confirmed capture of 32-pixel LEDs lands in the "
               "AREA clamp bucket -- 31, 'at least 31', which is the honest "
               "reading for the histogram and an unusable one for a ceiling");
            ck(e.led_max_h == 4,
               "...while the HEIGHT of the same blobs is 4 and nowhere near "
               "its own clamp: one axis saturating says nothing about the "
               "other, which is what makes the advice in the refusal below "
               "worth printing");
        }
        wiicam_cam_command("cam=bhmax:0,pxmax:0");
        g_replies.clear();
        wiicam_cam_command("cam=pxmax:32");
        ck(!g_replies.empty()
           && g_replies[0] == "CAM: pxmax not set -- this rig's LED blobs are "
                              "larger than the pixel measurement can express, "
                              "so no safe ceiling can be derived for it. Use "
                              "bhmax, which camfit measures properly\n",
           "pxmax:32 IS REFUSED against that live capture, and the refusal "
           "names the reason rather than a figure: before the fix the clamp "
           "was handed back as a maximum, 32 cleared it, and the gate cut this "
           "rig's own 32-pixel LEDs out of every frame it saw");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("pxmax=0 ") != std::string::npos,
           "...and nothing was written");
        g_replies.clear();
        wiicam_cam_command("cam=pxmax:0");
        ck(g_replies.size() == 1
           && g_replies[0].find("not set") == std::string::npos,
           "pxmax:0 still lands: a floor that has given up on naming a number "
           "must not also take away the value that turns the gate off");
        g_replies.clear();
        wiicam_cam_command("cam=bhmax:4");
        ck(g_replies.size() == 1
           && g_replies[0].find("not set") == std::string::npos
           && g_replies[0].find("bhmax=4 ") != std::string::npos,
           "...and bhmax:4 lands on the very same capture, so 'Use bhmax' is "
           "advice the user can act on -- the height histogram measured these "
           "blobs properly and the pixel one could not");
        wiicam_cam_command("cam=bhmax:0,pxmax:0,armax:0,bmin:0,bmax:15,rtol:0");
        wiicam_cam_command("camlearn=on:0");
        wiicam_cam_command("camlearn=reset");
        wiicam_cam_command("cam=mirx:1");
    }

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
    //
    // The SHAPE gate does the rejecting here, because the block also asserts
    // on bnear, and bnear counts shape-gate rejections only -- it is the
    // number the tools attach "bhmax:0 turns it off" to, and that advice is
    // only true of the gate it counts. The learning property itself holds for
    // any gate (the size-window case is pinned further down).
    {
        arm_clean();
        wiicam_cam_command("cam=bhmax:5");
        FullObj junk[4];
        memcpy(junk, RIG, sizeof(junk));
        junk[3].ymx = junk[3].ymn + 9;       // a real corner, grown to 9 rows
        const unsigned long rej_a  = blobstat("bsrej=");
        const unsigned long r3a    = blobstat("br3=");
        const unsigned long near_a = blobstat("bnear=");
        for (int i = 0; i < 6; ++i) frame(junk);
        const unsigned long rej_b  = blobstat("bsrej=");
        const unsigned long r3b    = blobstat("br3=");
        ck(rej_b >= rej_a + 6,
           "the shape gate really did reject a blob in every one of these "
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
        wiicam_cam_command("cam=bhmax:0,bmin:0,bmax:15");
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

    // ---- the negative label is POSITIONAL, and no gate votes in it ---------
    // The block above walked the same branch through three different gates.
    // This one says why that was possible: the label is not about gates at
    // all. On a frame the resolver LOCKED with three real corners and a
    // reconstructed fourth, every blob is measured against the four resolved
    // corners, and one further than TWICE the association radius from every
    // one of them is the stray -- kept or rejected, by any knob or by none.
    //
    // Distances are derived from quad_default_config().gate rather than
    // written down. The radius is the resolver's to choose, and a test holding
    // its own copy of the number stops testing the rule the day it moves.
    {
        // A distance in the resolver's 240x176 space, converted back into the
        // sensor's native x step. mirx is on, so a blob moved to a LOWER raw x
        // sits at a HIGHER normalised x.
        const float GATE2   = 2.0f * quad_default_config().gate;
        const float PX_PER  = WIICAM_W / WIICAM_NORM_W;
        const int   DX_IN   = (int)((GATE2 - 2.0f) * PX_PER + 0.5f);
        const int   DX_OUT  = (int)((GATE2 + 2.0f) * PX_PER + 0.5f);

        // Every test below starts from a clean lock on four real corners, with
        // the capture OFF so the lock-in itself is never learned from. Without
        // this each case would inherit the resolver state the last one left,
        // and a resolver that had lost its model reports two real corners for
        // a frame that plainly has three -- which looks exactly like the
        // feature being broken.
        auto relock = []() {
            wiicam_cam_command("cam=fmt:2,bmin:0,bmax:15,rtol:0,"
                               "bhmax:0,pxmax:0,armax:0");
            wiicam_cam_command("camlearn=on:0");
            for (int i = 0; i < 12; ++i) frame(SHAPE_RIG);
        };

        // ---- THE REGRESSION: no gate at all, and a sample is still taken ---
        // Every gate at 0 is what SHIPS. The first version of this rule
        // required the size gate to have rejected the blob, so on a gun
        // straight out of the box nothing was ever rejected, the negative
        // class stayed structurally empty, and 'camfit' -- the only thing that
        // would have set a gate in the first place -- could never reach a
        // verdict. The module would have looked like it was working.
        //
        // The frame here has no gate switched on anywhere. The stray arrives
        // as an ordinary KEPT blob that the resolver simply declines to
        // associate, which is how a bright window actually presents itself on
        // a sensor with four object slots: it DISPLACES an LED rather than
        // being refused entry.
        {
            relock();
            arm_clean();
            const unsigned long rej_a  = blobstat("brej=");
            const unsigned long srej_a = blobstat("bsrej=");
            const unsigned long near_a = blobstat("bnear=");
            const unsigned long far_a  = blobstat("bfar=");
            const unsigned long r3a    = blobstat("br3=");
            frame(SHAPE_STRAY);
            ck(blobstat("brej=") == rej_a && blobstat("bsrej=") == srej_a,
               "not one gate rejected anything in this frame -- the size "
               "window wide open and all three shape knobs at 0, exactly as a "
               "gun ships");
            ck(blobstat("br3=") == r3a + 1,
               "...and the resolver still saw three real corners and "
               "reconstructed the fourth: the stray took a slot, it was not "
               "gated out of one");
            ck(wl_blobs(1) == 1u,
               "AND A NEGATIVE SAMPLE IS RECORDED ANYWAY. This is the "
               "regression: the old rule needed a gate rejection, got none on "
               "a shipped gun, and left the negative class permanently empty "
               "-- a dead feature that reported itself as a working one");
            ck(blobstat("bfar=") == far_a,
               "...and bfar does NOT move. It counts strays the SHAPE GATE "
               "refused and the resolver then vouched for -- a kept stray is "
               "a sample for the negative class, not a rejection anyone can "
               "take credit for. It used to tick here, and that put kept "
               "strays into the number the tools subtract from bsrej: one "
               "kept stray a frame cancelled the whole 'unexplained "
               "rejections' threshold and masked a gate eating a real LED. "
               "The negative class filling is reported by camfit's straym, "
               "which reads wl_blobs(1) directly");
            ck(blobstat("bnear=") == near_a,
               "...while bnear does not move: nothing was rejected, so there "
               "is no gate verdict to hold against the gate");
            ck(wl_blobs(0) == 0u && wl_frames() == 0u,
               "...and nothing at all reaches the positive class: three real "
               "corners is not four, and only four says every blob in the "
               "frame was an LED");
            ck(hbin(1, WL_BW) == 5 && hbin(1, WL_BH) == 2,
               "...with the sample carrying the STRAY's own 5x2 box rather "
               "than a corner's, which is what says the right blob of the four "
               "was the one picked out");
        }

        // ---- the negative class and bfar are two different counts ----------
        // wl_blobs(1) is samples LEARNED; bfar is shape-gate rejections the
        // resolver VOUCHED for. With no gate they diverge completely: three
        // stray frames are three samples and zero on bfar. And with the gate
        // on they move together, because now the stray IS a shape rejection
        // and the resolver still places it far. Both halves are pinned,
        // because a counter that tracked the samples one-for-one was the
        // previous behaviour and it read as correct right up until the tools
        // subtracted it from a number it was no longer a subset of.
        {
            relock();
            arm_clean();
            const unsigned long far_a = blobstat("bfar=");
            for (int i = 0; i < 3; ++i) frame(SHAPE_STRAY);
            ck(wl_blobs(1) == 3u && blobstat("bfar=") == far_a,
               "three KEPT stray frames are three negative samples and not one "
               "tick on bfar: nothing was refused, so nothing is vouched for");

            relock();
            arm_clean();
            wiicam_cam_command("cam=armax:16");        // the 2.5:1 stray is refused
            const unsigned long far_b  = blobstat("bfar=");
            const unsigned long srej_b = blobstat("bsrej=");
            for (int i = 0; i < 3; ++i) frame(SHAPE_STRAY);
            ck(wl_blobs(1) == 3u && blobstat("bfar=") == far_b + 3
               && blobstat("bsrej=") == srej_b + 3,
               "...and the same three frames with the shape gate refusing the "
               "stray are three samples AND three on bfar AND three on bsrej: "
               "every refusal vouched for, which is what 'bsrej - bfar - "
               "bnear == 0' has to mean when the gate is doing its job");
            wiicam_cam_command("cam=armax:0");
        }

        // ---- the counters run with the capture OFF -------------------------
        // This was a blocker. The classification and the two counters used to
        // sit inside 'if (wl_enabled())' with the recording, so during
        // ordinary play -- capture off, which is how a gun ships and how the
        // tools leave it -- bfar and bnear were frozen at zero while bsrej
        // climbed. The tools' "unexplained rejections" warning subtracts them
        // from bsrej, so it degenerated to the raw count it was written to
        // replace, and the false-negative meter was dead in exactly the
        // sessions where a wrong gate does its damage.
        {
            relock();                                  // leaves the capture OFF
            wiicam_cam_command("cam=armax:16");
            ck(!wl_enabled(), "the capture is off for this block");
            const unsigned long far_a  = blobstat("bfar=");
            const unsigned long srej_a = blobstat("bsrej=");
            const unsigned long neg_a  = wl_blobs(1);
            frame(SHAPE_STRAY);
            ck(blobstat("bsrej=") == srej_a + 1 && blobstat("bfar=") == far_a + 1,
               "capture OFF, shape gate refuses the stray, resolver places it "
               "far: bfar ticks. It did not, before -- the counter only ran "
               "while a capture was armed, so a gate refusing the sun all "
               "evening looked to the tools like a gate nobody could vouch "
               "for");
            ck(wl_blobs(1) == neg_a,
               "...and NOTHING is recorded into the histograms, because the "
               "capture is off: counting and learning are two different "
               "things and only the second one is armed");

            wiicam_cam_command("cam=armax:0,bhmax:5");  // the 3..5-row corners pass, a 9-row one does not
            FullObj onspot[4];
            memcpy(onspot, SHAPE_RIG, sizeof(onspot));
            onspot[3].ymx = onspot[3].ymn + 9;          // a real corner, grown tall
            const unsigned long near_a = blobstat("bnear=");
            frame(onspot);
            ck(blobstat("bnear=") == near_a + 1,
               "...and bnear ticks with the capture off too, when the shape "
               "gate refuses a blob sitting where the missing LED had to be");
            wiicam_cam_command("cam=bhmax:0");
        }

        // ---- two far blobs: no sample, and worth being exact about why -----
        // The guard reads like a tie-break and is not one. This sensor has
        // four object slots; a blob the resolver ASSOCIATED is within one
        // association radius of its corner and therefore can never be more
        // than two away from it; so three real corners leave at most one blob
        // in the frame that can be far. A second far blob is a third corner
        // the resolver did not get, and the frame comes back with TWO real
        // corners and never reaches the label at all.
        //
        // Which is the point: the frame is not confirmed, so nothing is
        // learned from it -- the safety property, arrived at from the
        // direction that looks most like an exception to it. The nfar == 1
        // guard is what keeps that true if the association radius, the slot
        // count or the reconstruction ever change, because a label drawn from
        // a geometry we do not trust is worse than no label at all.
        {
            relock();
            arm_clean();
            FullObj two[4];
            memcpy(two, SHAPE_RIG, sizeof(two));
            // Both bottom corners pulled in toward the middle of the rig, each
            // by just over twice the radius, so each is far from its OWN
            // corner and further still from the other three.
            two[2].x = SHAPE_RIG[2].x + DX_OUT;
            two[3].x = SHAPE_RIG[3].x - DX_OUT;
            const unsigned long r3a   = blobstat("br3=");
            const unsigned long r2a   = blobstat("br2=");
            const unsigned long far_a = blobstat("bfar=");
            frame(two);
            ck(blobstat("br2=") == r2a + 1 && blobstat("br3=") == r3a,
               "two far blobs cost the resolver a second corner: the frame "
               "comes back with TWO real corners, not the three the negative "
               "label needs");
            ck(wl_blobs(1) == 0u && blobstat("bfar=") == far_a,
               "...so nothing is learned and nothing is counted -- a frame the "
               "resolver could not confirm teaches the sink nothing, however "
               "obviously strayish the blobs in it look");
            ck(wl_blobs(0) == 0u && wl_frames() == 0u,
               "...and least of all in the positive class");
        }

        // ---- just inside twice the radius, then just outside it ------------
        // TWICE, and not once. Being wrong here poisons the negative
        // distribution with real LEDs and teaches the gate to reject them, so
        // the bar for calling something a stray is deliberately higher than
        // the bar the resolver uses for calling something a corner. Both
        // blobs below are already well past the resolver's OWN radius -- both
        // frames come back with three real corners, so neither blob was
        // associated -- and the only thing that separates them is the
        // doubling.
        {
            relock();
            arm_clean();
            FullObj inside[4];
            memcpy(inside, SHAPE_RIG, sizeof(inside));
            inside[3].x = SHAPE_RIG[3].x - DX_IN;
            const unsigned long r3a   = blobstat("br3=");
            const unsigned long far_a = blobstat("bfar=");
            frame(inside);
            ck(blobstat("br3=") == r3a + 1,
               "a blob two pixels INSIDE the doubled radius is still far "
               "enough that the resolver refuses to associate it -- three real "
               "corners, so the label is available and the frame really did "
               "reach the test");
            ck(wl_blobs(1) == 0u && blobstat("bfar=") == far_a,
               "...and it is NOT learned as a stray: inside twice the radius "
               "is where a real corner the resolver merely mis-tracked lives, "
               "and filing one of those under 'stray' teaches the gate to "
               "throw away LEDs");

            relock();
            arm_clean();
            FullObj outside[4];
            memcpy(outside, SHAPE_RIG, sizeof(outside));
            outside[3].x = SHAPE_RIG[3].x - DX_OUT;
            const unsigned long r3b   = blobstat("br3=");
            const unsigned long far_b = blobstat("bfar=");
            frame(outside);
            ck(blobstat("br3=") == r3b + 1,
               "four pixels further out is the same three-real-corner frame");
            ck(wl_blobs(1) == 1u && blobstat("bfar=") == far_b,
               "...and NOW it is a stray: the two frames differ by four "
               "normalised pixels either side of twice the association radius, "
               "which is the whole of the rule. bfar stays put -- no gate "
               "refused it, so there is nothing to vouch for");
        }

        // ---- three corners and no stray at all -----------------------------
        // The label is available and there is nothing to apply it to. A frame
        // that merely LOST an LED must not manufacture a negative sample out
        // of the corner that is missing -- there is no blob there to measure,
        // and inventing one would fill the stray distribution with the shape
        // of whatever the reconstruction happened to sit on.
        {
            relock();
            arm_clean();
            int p3[4] = { SHAPE_RIG[0].x, SHAPE_RIG[1].x, SHAPE_RIG[2].x, 0 };
            int y3[4] = { SHAPE_RIG[0].y, SHAPE_RIG[1].y, SHAPE_RIG[2].y, 0 };
            int s3[4] = { SHAPE_RIG[0].sz, SHAPE_RIG[1].sz, SHAPE_RIG[2].sz, -1 };
            float sx = 0.0f, sy = 0.0f;
            const unsigned long r3a   = blobstat("br3=");
            const unsigned long far_a = blobstat("bfar=");
            for (int i = 0; i < 3; ++i) {
                p3[0] += (i & 1) ? 1 : -1;
                g_t += DT;
                wiicam_aim_process_sz(p3, y3, s3, 0x7, g_t, &sx, &sy);
            }
            ck(blobstat("br3=") == r3a + 3,
               "three frames with only three blobs in them, each resolved with "
               "a reconstructed fourth corner");
            ck(wl_blobs(1) == 0u && blobstat("bfar=") == far_a,
               "...and not one negative sample between them: with nothing in "
               "the frame further than twice the radius there is no stray to "
               "learn, and the missing corner is not a blob");
            ck(wl_blobs(0) == 0u && wl_frames() == 0u,
               "...nor a positive one, because three real corners is not four");
        }

        // ---- bnear: the meter for a gate that is too TIGHT -----------------
        // A rejection ON the reconstructed corner is the opposite reading from
        // a rejection away from it: the gate almost certainly threw away a
        // real LED. That is evidence about the GATE, counted where a user can
        // see it, and it must never become a negative SAMPLE -- a gate
        // teaching itself that LEDs are strays is the feedback loop the whole
        // module is arranged to refuse.
        //
        // SHAPE gate only. bnear is what the tools subtract from bsrej, and it
        // is what their "bhmax:0 turns it off" advice is attached to; both are
        // true only if it counts the same gate bsrej does. It used to count a
        // rejection by ANY gate, so an rtol or size-window drop on a corner
        // sent the user to switch off a height gate that was not the one
        // doing it. The size window has its own too-tight signal, bvalve.
        {
            relock();
            arm_clean();
            wiicam_cam_command("cam=bhmax:5");
            FullObj onspot[4];
            memcpy(onspot, SHAPE_RIG, sizeof(onspot));
            onspot[3].ymx = onspot[3].ymn + 9;     // a real corner, grown to 9 rows
            const unsigned long near_a = blobstat("bnear=");
            const unsigned long far_a  = blobstat("bfar=");
            const unsigned long srej_a = blobstat("bsrej=");
            frame(onspot);
            ck(blobstat("bsrej=") == srej_a + 1,
               "the SHAPE gate rejected exactly one blob, and it was sitting "
               "where the missing LED should be");
            ck(blobstat("bnear=") == near_a + 1,
               "...so bnear ticks once: the false-negative meter, which reads "
               "high long before a gate this tight shows up as a cursor that "
               "will not track");
            ck(wl_blobs(1) == 0u && blobstat("bfar=") == far_a,
               "...and NOTHING is learned from it -- the gate must never end "
               "up teaching itself that its own mistakes were strays");

            // The same gate, the same rejection count, with the rejected blob
            // moved off the corner. Now the rejection was RIGHT, and a meter
            // that ticked here too would read highest on the gates that work
            // best -- which is worse than not having it.
            relock();
            arm_clean();
            wiicam_cam_command("cam=bhmax:5");
            FullObj offspot[4];
            memcpy(offspot, SHAPE_STRAY, sizeof(offspot));
            offspot[3].ymx = offspot[3].ymn + 9;   // the stray, grown to 9 rows
            const unsigned long near_b = blobstat("bnear=");
            const unsigned long far_b  = blobstat("bfar=");
            const unsigned long srej_b = blobstat("bsrej=");
            frame(offspot);
            ck(blobstat("bsrej=") == srej_b + 1,
               "one blob rejected here as well, so the two cases differ only "
               "in WHERE the rejected blob was");
            ck(blobstat("bnear=") == near_b,
               "...and bnear does NOT move: the rejected blob was the far one, "
               "the gate was right, and a correct rejection is not evidence "
               "against the gate that made it");
            ck(wl_blobs(1) == 1u && blobstat("bfar=") == far_b + 1,
               "...it is a negative SAMPLE instead, counted on bfar -- the two "
               "counters partition the shape gate's rejections rather than "
               "double-counting them");

            // The SIZE WINDOW making the same on-corner rejection: brej ticks,
            // bnear does not. This is the narrowing, and the reason for it is
            // in the advice the tools attach to bnear -- "bhmax:0 turns it
            // off" -- which would have sent this user to switch off a height
            // gate that was not even on.
            relock();
            arm_clean();
            wiicam_cam_command("cam=bhmax:0,bmin:0,bmax:5");
            FullObj onspot2[4];
            memcpy(onspot2, SHAPE_RIG, sizeof(onspot2));
            onspot2[3].sz = 12;                    // a real corner, out of window
            const unsigned long near_c = blobstat("bnear=");
            const unsigned long rej_c  = blobstat("brej=");
            const unsigned long srej_c = blobstat("bsrej=");
            frame(onspot2);
            ck(blobstat("brej=") == rej_c + 1 && blobstat("bsrej=") == srej_c,
               "the size window rejected the on-corner blob, the shape gate "
               "did not");
            ck(blobstat("bnear=") == near_c,
               "...and bnear does NOT tick for it: the meter belongs to the "
               "shape gate, whose advice is the only advice attached to it. "
               "The window's own too-tight signal is bvalve");
            ck(wl_blobs(1) == 0u,
               "...and it is not learned either -- a corner is a corner "
               "whichever gate dropped it");
        }

        // Left ARMED on purpose: the serial block below opens by asserting
        // that an earlier capture is sitting in the sink waiting to be read,
        // which is what the arming command has to get rid of.
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

    // ---- camreset clears the capture and arms it again ------------------
    // It is the command a user reaches for when nothing works: what was
    // measured is about a gun that no longer exists, but the gun still has to
    // learn as you play from here (G3), so the capture comes back armed.
    {
        arm_clean();
        frame(RIG);
        ck(wl_enabled() == 1 && wl_frames() == 1u, "a capture is running");
        g_replies.clear();
        wiicam_cam_command("camreset");
        ck(wl_enabled() == 1, "camreset leaves the capture ARMED, as boot does");
        ck(wl_frames() == 0u && wl_blobs(0) == 0u,
           "...and CLEARED: the frame that was in it is gone. Stopping and "
           "keeping left the refusal floor reading the old bar's LEDs after "
           "the one command a user reaches for to start over");
        // camreset also puts the format back to basic, so drive the plain
        // entry point: the frames after it count from zero.
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
        ck(wl_frames() == 3u,
           "...and the frames after it are counted from zero (the first one "
           "re-seeds the resolver and is not learned), so what the capture "
           "holds is only ever about the gun as it is now");
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
//
// 3. `nfar >= 2` IS STILL UNREACHABLE, and the guard against it stays anyway.
//    The rewritten label requires exactly one blob further than twice the
//    association radius from every resolved corner. Two would have to arrive
//    in a frame the resolver confirmed at THREE real corners -- and it cannot,
//    on this sensor: there are four object slots, an associated blob is within
//    ONE radius of its corner and therefore never counts as far, so three real
//    corners leave at most one blob that can. A second far blob is a third
//    corner the resolver did not get, and the frame comes back with two real
//    corners and never reaches the label. "two far blobs: no sample" above
//    asserts the outcome and br2 rather than br3, which is the honest shape of
//    that test: it pins the safety property, not the tie-break. The guard is
//    what keeps the property true if the slot count, the association radius or
//    the reconstruction ever change under it.
