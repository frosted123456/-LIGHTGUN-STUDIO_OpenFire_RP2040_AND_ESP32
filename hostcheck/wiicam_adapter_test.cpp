// The RP2040/wiicam front end, on the host. The one property that must never
// regress: an unseen slot's STALE coordinates (which the DFRobot driver
// retains forever) must not reach the resolver. Plus: normalisation into
// 240x176, raw mode for lens sweeps, lock + solve through a real calibration,
// lead extrapolation, and the ~cam command subset.
//
// And the three things the sensor read itself now depends on: the three-value
// report format carried with its epoch in ONE word, the 37-byte full-mode
// report the vendored driver cannot read (so the unpack is ours, and testable
// here), and the blob gate that now survives a power cycle.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <utility>
#include <string>
#include "wiicam_aim.h"
#include "aim_runtime.h"
#include "nvs.h"

// ---- fake NVS (same shape as esp_link_test's) ----------------------------
static unsigned char g_blob[512]; static size_t g_bl = 0; static bool g_bh = false;
static unsigned char g_lens[64];  static size_t g_ll = 0; static bool g_lh = false;
static bool is_lens(const char* k){ return k && k[0]=='l' && k[1]=='e' && k[2]=='n'; }
extern "C" {
esp_err_t nvs_open(const char*, nvs_open_mode_t, nvs_handle_t* h){ *h=1; return ESP_OK; }
esp_err_t nvs_set_blob(nvs_handle_t, const char* k, const void* v, size_t n){
    if (is_lens(k)) { memcpy(g_lens,v,n); g_ll=n; g_lh=true; return ESP_OK; }
    memcpy(g_blob,v,n); g_bl=n; g_bh=true; return ESP_OK; }
esp_err_t nvs_get_blob(nvs_handle_t, const char* k, void* o, size_t* l){
    if (is_lens(k)) { if(!g_lh) return ESP_ERR_NVS_NOT_FOUND;
        if(*l<g_ll) { return -1; } memcpy(o,g_lens,g_ll); *l=g_ll; return ESP_OK; }
    if(!g_bh) return ESP_ERR_NVS_NOT_FOUND;
    if(*l<g_bl) { return -1; } memcpy(o,g_blob,g_bl); *l=g_bl; return ESP_OK; }
// The blob gate is a u32 under its own key, and camreset ERASES it -- so the
// stub has to know that key by name. Key-blind, an erase of "gate0" fell
// through to the calibration blob: camreset wiped the calibration and left the
// saved gate exactly where it was, which is both mistakes at once.
//
// And there are TWO of them now: the size window in "gate0", the shape gate in
// "gate1". They are separate on purpose -- the first word is full at 14 bits
// under its tag, and re-packing it would make every gate already in a gun's
// flash unreadable -- so the store they are tested against has to keep them
// separate too. A key-blind u32 slot hides the exact failure the split exists
// to prevent: camsave writes both, the second lands on top of the first, and
// the next boot reads the shape gate's payload as a size window.
static bool is_gate(const char* k){
    return k && (!strcmp(k, "gate0") || !strcmp(k, "gate1")); }
struct U32Slot { char key[16]; uint32_t v; bool have; };
static U32Slot g_u32s[4];       // gate0 gate1, plus room
static U32Slot* u32_find(const char* k){
    for (auto& s : g_u32s) if (s.have && k && !strcmp(s.key, k)) return &s;
    return nullptr; }
// Dispatched by NAME, not by what happens to be present: an erase of a gate
// key that is already absent must still be an erase of THAT key, not a
// fall-through to the calibration blob.
//
// And an absent key answers ESP_ERR_NVS_NOT_FOUND, exactly as the real store
// does. Answering OK for everything made "clearing a key that was never
// written is not a failure" a test of nothing at all -- the one line it exists
// to cover was never reached, and that line runs on every camreset on every
// gun that has saved no gate.
esp_err_t nvs_erase_key(nvs_handle_t, const char* k){
    if(is_gate(k)) {
        U32Slot* s = u32_find(k);
        if(!s) return ESP_ERR_NVS_NOT_FOUND;
        s->have = false; return ESP_OK; }
    if(is_lens(k)) {
        if(!g_lh) return ESP_ERR_NVS_NOT_FOUND;
        g_lh = false; return ESP_OK; }
    if(!g_bh) return ESP_ERR_NVS_NOT_FOUND;
    g_bh = false; return ESP_OK; }
// key-aware i16 store: lead and smoothing live under their own keys
struct I16Slot { char key[16]; int16_t v; bool have; };
static I16Slot g_i16s[8];   // lead0 smth0 dead0 tmod0 fir0, plus room
static I16Slot* i16_find(const char* k){
    for (auto& s : g_i16s) if (s.have && !strcmp(s.key, k)) return &s;
    return nullptr; }
esp_err_t nvs_set_i16(nvs_handle_t, const char* k, int16_t v){
    if (I16Slot* s = i16_find(k)) { s->v = v; return ESP_OK; }
    for (auto& s : g_i16s)
        if (!s.have) { strncpy(s.key, k, 15); s.v = v; s.have = true; return ESP_OK; }
    return -1; }
esp_err_t nvs_get_i16(nvs_handle_t, const char* k, int16_t* v){
    if (I16Slot* s = i16_find(k)) { *v = s->v; return ESP_OK; }
    return ESP_ERR_NVS_NOT_FOUND; }
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
// register writes the firmware hook would have made
static std::vector<std::pair<int,int> > g_reg;
static bool g_reg_fail = false;   // make the hook refuse, as a dead camera does
static void line_sink(const char* s){ g_lines.push_back(s); }
static void reply_sink(const char* s){ g_replies.push_back(s); }

// A reply sink that publishes a NEW camera frame from inside the first of the
// two '~camblob?' lines -- which is exactly where the poll core lands on the
// gun. The two replies are emitted back to back, but nothing stops the camera
// between them.
static bool     g_race_arm = false;
static int      g_race_px[4], g_race_py[4], g_race_sz[4];
static unsigned g_race_seen = 0;
static uint64_t g_race_t = 0;
static void reply_sink_racing(const char* s)
{
    g_replies.push_back(s);
    // Line 1 is "CAM: blob fmt=..."; line 2 is "CAM: blobs ...".
    if (g_race_arm && !strncmp(s, "CAM: blob ", 10)) {
        g_race_arm = false;
        float rx = 0.0f, ry = 0.0f;
        wiicam_aim_process_sz(g_race_px, g_race_py, g_race_sz, g_race_seen,
                              g_race_t, &rx, &ry);
    }
}

static int t_sens = 0;
static int t_sens_saved = 0;
static int fails = 0;
static void ck(bool ok, const char* m){ printf("  [%s] %s\n", ok?"PASS":"FAIL", m); if(!ok) fails++; }

// A rig rectangle in wiicam units, centred, and its per-slot report.
static void rig(int px[4], int py[4], float cx, float cy, float w, float h)
{
    px[0]=(int)(cx-w/2); py[0]=(int)(cy-h/2);
    px[1]=(int)(cx+w/2); py[1]=(int)(cy-h/2);
    px[2]=(int)(cx-w/2); py[2]=(int)(cy+h/2);
    px[3]=(int)(cx+w/2); py[3]=(int)(cy+h/2);
}

// ---- a fake full-mode bus transaction -------------------------------------
// The vendored driver declares this format and cannot read it -- its receive
// union is 13 bytes and a full report is 37 -- so the transaction is a hook and
// the unpack lives in our code, which is what makes it testable here at all.
//
// One object, as the sensor lays it out: xLow, yLow, then ONE byte carrying the
// two high bits of x, the two high bits of y AND the 4-bit size, then the four
// 7-bit box corners, a reserved byte, and the 8-bit intensity.
struct FullObj { int x, y, sz, xmn, ymn, xmx, ymx, inten; };
static FullObj g_fobj[4];
static int g_fcalls    = 0;   // how many reads the firmware asked for
static int g_flen      = 0;   // and how many bytes it asked for
static int g_ffail_on  = 0;   // make the Nth read fail, as a dead bus does
static int g_fdrift    = 0;   // a different frame on every read (a fast pan)
static int g_fhdrdrift = 0;   // only the HEADER byte differs between reads

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
    ++g_fcalls;
    g_flen = len;
    if (g_ffail_on == g_fcalls) return 0;
    if (len < WIICAM_FULL_LEN) return 0;   // a short read is a failed read
    FullObj o[4];
    memcpy(o, g_fobj, sizeof(o));
    // A fast pan is exactly when two reads disagree, and exactly when the gun
    // can least afford to be handed nothing.
    if (g_fdrift) o[0].x = (o[0].x + g_fcalls) & 0x3FF;
    full_pack(buf, (unsigned char)(g_fhdrdrift ? g_fcalls : 0x00), o);
    return 1;
}

int main()
{
    printf("wiicam adapter\n\n");
    aim_runtime_begin();
    wiicam_aim_begin();
    wiicam_set_line_sink(line_sink);
    wiicam_set_reply_sink(reply_sink);
    wiicam_set_sens_hooks([](int v){ t_sens = v; }, [](){ return t_sens; },
                          [](){ t_sens_saved++; });

    int px[4], py[4];
    uint64_t t = 1000000;
    const uint64_t DT = 4785;              // ~209 Hz, like the stock poll timer
    float sx, sy;

    // ---- raw mode + normalisation ----------------------------------------
    wiicam_cam_command("cam=res:0,dash:2,dashhz:0");
    rig(px, py, 512, 384, 512, 288);
    g_lines.clear();
    wiicam_aim_process(px, py, 0xF, t, &sx, &sy);
    ck(g_lines.size() == 1, "raw mode emits a Q line");
    int qx[4], qy[4], n = -1; unsigned long ms;
    if (!g_lines.empty())
        sscanf(g_lines[0].c_str(), "Q,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               &ms, &n, &qx[0],&qy[0],&qx[1],&qy[1],&qx[2],&qy[2],&qx[3],&qy[3]);
    ck(n == 4, "with all four slots seen, count is 4");
    // X is UN-MIRRORED by default: native 256 -> (1023-256)*240/1024*10 = 1798
    ck(n == 4 && qx[0] == 1798, "x un-mirrored and normalised (tenths)");
    // 384-144=240 native -> *176/768*10 = 550 tenths (Y untouched)
    ck(n == 4 && qy[0] == 550, "y normalised into 176-space (tenths)");
    // a byte-identical poll is the previous camera frame seen again: skipped
    g_lines.clear();
    t += DT;
    wiicam_aim_process(px, py, 0xF, t, &sx, &sy);
    ck(g_lines.empty(), "duplicate report is skipped, no Q re-emit");
    // the mirror is the difference between a valid rectangle and the negative
    // width the first bench calibration produced -- prove it flips
    wiicam_cam_command("cam=mirx:0");
    g_lines.clear();
    t += DT;
    wiicam_aim_process(px, py, 0xF, t, &sx, &sy);
    if (!g_lines.empty())
        sscanf(g_lines[0].c_str(), "Q,%lu,%d,%d,%d", &ms, &n, &qx[0], &qy[0]);
    ck(qx[0] == 600, "mirx:0 restores the raw orientation");
    wiicam_cam_command("cam=mirx:1");
    t += DT;

    // ---- THE STALE-SLOT TRAP ---------------------------------------------
    // Same arrays (the driver keeps old values), but slot 2 unseen.
    g_lines.clear();
    t += DT;
    wiicam_aim_process(px, py, 0xF & ~(1u<<2), t, &sx, &sy);
    n = -1;
    if (!g_lines.empty())
        sscanf(g_lines[0].c_str(), "Q,%lu,%d", &ms, &n);
    ck(n == 3, "an unseen slot's stale coordinates are MASKED, not forwarded");

    // ---- resolver + solve through a real calibration ---------------------
    ck(aim_runtime_command("aimcal=0.5,0.5,0.35,1.28,0.0,0.0"), "install a calibration");
    wiicam_cam_command("cam=res:2,dash:0");
    bool solved = false;
    for (int i = 0; i < 40; ++i) {         // let the resolver lock
        t += DT;
        // integer jitter: a real sensor never repeats byte-identical frames,
        // and the duplicate cache skips exact repeats by design
        px[1] += (i & 1) ? 1 : -1;
        solved = wiicam_aim_process(px, py, 0xF, t, &sx, &sy);
    }
    ck(solved, "locked resolver + active calibration produce a position");
    ck(fabsf(sx - 0.5f) < 0.02f && fabsf(sy - 0.5f) < 0.06f,
       "aiming at the rig centre lands near screen centre");

    // one corner lost mid-run: the resolver reconstructs, aim continues
    t += DT;
    solved = wiicam_aim_process(px, py, 0xF & ~(1u<<3), t, &sx, &sy);
    ck(solved, "a dropped corner is reconstructed; aim does not stop");

    // ---- latency lead -----------------------------------------------------
    wiicam_cam_command("cam=lead:20,dash:2,dashhz:0");
    for (int step = 0; step < 30; ++step) {    // steady rightward pan
        for (int i = 0; i < 4; ++i) px[i] += 4;
        t += DT;
        g_lines.clear();
        wiicam_aim_process(px, py, 0xF, t, &sx, &sy);
    }
    int lx = 0;
    if (!g_lines.empty())
        sscanf(g_lines[0].c_str(), "Q,%lu,%d,%d", &ms, &n, &lx);
    // where the raw corner sits in 240-space tenths, no lead
    const int raw_x = (int)lroundf(px[0] * (240.0f/1024.0f) * 10.0f);
    ck(lx > raw_x + 5, "published quad leads the raw position during a pan");

    // ---- command surface --------------------------------------------------
    g_replies.clear();
    wiicam_cam_command("cam?");
    ck(!g_replies.empty() && g_replies[0].find("board=rp2040-wiicam") != std::string::npos,
       "cam? names the board");
    wiicam_cam_command("cam=lens:2,lfeq:900,lfpx:1847,lcxu:32,lcyu:-15");
    // smoothing goes through the shared runtime knob
    const float fc_before = aim_filter_min_cutoff();
    wiicam_cam_command("cam=smooth:8");
    ck(aim_smooth_get() == 8 && aim_filter_min_cutoff() < fc_before,
       "smooth key maps to a heavier One Euro pair");
    g_replies.clear();
    wiicam_cam_command("camsave");
    ck(g_lh, "camsave persisted the lens through the (fake) store");
    int16_t i16v = 0;
    ck(nvs_get_i16(1, "lead0", &i16v) == ESP_OK && i16v == 20,
       "camsave persisted the lead");
    ck(nvs_get_i16(1, "smth0", &i16v) == ESP_OK && i16v == 8,
       "camsave persisted the smoothing level under its own key");
    wiicam_cam_command("cam=dead:24");
    ck(aim_dead_get() == 24, "dead key reaches the shared runtime");
    wiicam_cam_command("camsave");
    ck(nvs_get_i16(1, "dead0", &i16v) == ESP_OK && i16v == 24,
       "camsave persisted the dead-band under its own key");
    g_replies.clear();
    wiicam_cam_command("cam?");
    ck(!g_replies.empty() && g_replies[0].find("dead=24") != std::string::npos,
       "cam? reports the dead-band");
    wiicam_cam_command("cam=dead:0");
    wiicam_cam_command("cam=sens:2");
    ck(t_sens == 2, "sens goes through the OpenFIRE hook");
    const int saves = t_sens_saved;
    wiicam_cam_command("camsave");
    ck(t_sens_saved == saves + 1,
       "camsave persists sensitivity via OpenFIRE's own prefs write");
    // reload path
    aim_smooth_set(3);                     // forget the live level...
    wiicam_aim_begin();
    g_replies.clear();
    wiicam_cam_command("cam?");
    ck(!g_replies.empty() && g_replies[0].find("lens=2") != std::string::npos,
       "a reboot restores the stored lens");
    ck(!g_replies.empty() && g_replies[0].find("lcxu=32") != std::string::npos
       && g_replies[0].find("lcyu=-15") != std::string::npos,
       "...including the distortion-centre offset");
    ck(aim_smooth_get() == 8, "...and a reboot restores the stored smoothing");

    // ---- the blob size gate (ambient light) ------------------------------
    // A window in the sensor's view does not ADD a fifth blob: the wiicam
    // reports four slots, so a bright patch TAKES one and an LED goes missing.
    // The only hardware fact that separates them is blob size, and only the
    // extended data format carries it. Everything here must be inert until
    // asked for, and must never be able to blind the gun.
    {
        wiicam_aim_begin();
        wiicam_cam_command("cam=res:0,dash:2,dashhz:0,mirx:1");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("fmt=0") != std::string::npos
           && g_replies[0].find("ext=0") != std::string::npos
           && g_replies[0].find("bmin=0") != std::string::npos
           && g_replies[0].find("bmax=15") != std::string::npos,
           "the gate ships inert: basic format, window wide open -- the "
           "sensor is read exactly as it was before this code existed");

        rig(px, py, 512, 384, 512, 288);
        int sz[4] = {3, 3, 3, 14};          // slot 3 is a big diffuse patch
        // Sizes are only believed when the caller actually has them: the plain
        // entry point passes none, and the gate must not act on a guess.
        wiicam_cam_command("cam=bmin:1,bmax:8");
        g_lines.clear();
        t += DT;
        wiicam_aim_process(px, py, 0xF, t, &sx, &sy);
        n = -1;
        if (!g_lines.empty()) sscanf(g_lines[0].c_str(), "Q,%lu,%d", &ms, &n);
        ck(n == 4, "with no sizes available the gate cannot drop anything");

        // With sizes, the oversized blob is dropped before the resolver.
        g_lines.clear();
        t += DT;
        px[0] += 1;                          // defeat the duplicate cache
        wiicam_aim_process_sz(px, py, sz, 0xF, t, &sx, &sy);
        n = -1;
        if (!g_lines.empty()) sscanf(g_lines[0].c_str(), "Q,%lu,%d", &ms, &n);
        ck(n == 3, "an out-of-window blob is dropped before the resolver");

        g_replies.clear();
        wiicam_cam_command("camblob?");
        ck(g_replies.size() >= 2
           && g_replies[0].find("brej=1") != std::string::npos,
           "camblob? counts what the gate dropped");
        ck(g_replies.size() >= 2
           && g_replies[1].find(",14,0") != std::string::npos,
           "and shows the offending blob with its size and REJECTED flag");

        // The floor. Below TWO points the resolver cannot fit anything and
        // GetPosition falls through to the stock, uncalibrated path -- the
        // cursor jumps. So a window that would reject everything gives the
        // least-offending blobs back instead of blinding the gun.
        wiicam_cam_command("cam=bmin:12,bmax:13");
        g_lines.clear();
        t += DT;
        px[0] += 1;
        wiicam_aim_process_sz(px, py, sz, 0xF, t, &sx, &sy);
        n = -1;
        if (!g_lines.empty()) sscanf(g_lines[0].c_str(), "Q,%lu,%d", &ms, &n);
        ck(n == 2, "a window that would reject EVERY blob is held to a floor "
                   "of two, never zero");
        g_replies.clear();
        wiicam_cam_command("camblob?");
        ck(!g_replies.empty()
           && g_replies[0].find("bvalve=") != std::string::npos
           && g_replies[0].find("bvalve=0 ") == std::string::npos,
           "and it SAYS the floor had to intervene -- a window this wrong used "
           "to report brej=0 and look idle");
        {
            unsigned long rej = 0;
            const char* a = strstr(g_replies[0].c_str(), "brej=");
            if (a) sscanf(a, "brej=%lu", &rej);
            ck(rej > 0,
               "and counts what the window WANTED to reject, not what survived "
               "the floor -- the old order reported zero in exactly the case "
               "the number exists to reveal");
        }

        // Clamping and the ordering guard.
        wiicam_cam_command("cam=bmin:0,bmax:99");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty() && g_replies[0].find("bmax=15") != std::string::npos,
           "sizes clamp to the sensor's own 0..15 range");
        // Order independence. The tools send one key per keystroke, so
        // clamping bmin against the CURRENT bmax as it arrives made the result
        // depend on which end was typed first: asking for 8..12 from a stored
        // 0..3 quietly produced 3..12, and the spin box then showed the value
        // being refused with no explanation.
        wiicam_cam_command("cam=bmin:0,bmax:3");
        wiicam_cam_command("cam=bmin:8");
        wiicam_cam_command("cam=bmax:12");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty() && g_replies[0].find("bmin=8") != std::string::npos
           && g_replies[0].find("bmax=12") != std::string::npos,
           "the two ends can be typed in either order and both land");
        // A window whose UPPER end is zero admits no size the sensor can
        // report, so it rejects every blob. The give-back floor keeps the gun
        // aiming, but on two blobs chosen for being least wrong -- and now
        // that the gate PERSISTS, one fat-finger on a spinner that starts at
        // 0, followed by any calibration, writes that to flash and it comes
        // back on every boot. Refused by name, the same way hwmax:0 is.
        wiicam_cam_command("cam=bmin:0,bmax:15");
        g_replies.clear();
        wiicam_cam_command("cam=bmax:0");
        ck(!g_replies.empty()
           && g_replies[0].find("rejects every blob") != std::string::npos,
           "bmax:0 from a wide-open window is refused BY NAME, not obeyed");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bmin=0 ") != std::string::npos
           && g_replies[0].find("bmax=15") != std::string::npos,
           "...and NEITHER end moves -- a refusal that half-applied would "
           "leave behind the window it was refusing to make");
        // But 0 is a perfectly good LOW end. The tools send one key per
        // keystroke, so bmax:0 arriving after bmin:5 is the legitimate window
        // 0..5 typed high-end-first, and the gate orders the ends where it
        // uses them. Refusing that would make a reachable window reachable
        // from one direction only -- the same asymmetry the ordering guard
        // above exists to kill.
        wiicam_cam_command("cam=bmin:5");
        g_replies.clear();
        wiicam_cam_command("cam=bmax:0");
        ck(g_replies.empty()
           || g_replies[0].find("rejects every blob") == std::string::npos,
           "bmax:0 under a bmin of 5 is not refused");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bmin=5") != std::string::npos
           && g_replies[0].find("bmax=0") != std::string::npos,
           "...it lands, and the gate reads it low-to-high as the window 0..5");
        // And an inverted window still gates sanely rather than dropping
        // everything: it is ordered where it is USED.
        wiicam_cam_command("cam=bmin:9,bmax:2");
        rig(px, py, 512, 384, 512, 288);
        int sz2[4] = {1, 5, 5, 14};        // 5s inside 2..9, the others outside
        wiicam_cam_command("cam=res:0,dash:2,dashhz:0");
        g_lines.clear();
        t += DT;
        px[0] += 1;
        wiicam_aim_process_sz(px, py, sz2, 0xF, t, &sx, &sy);
        n = -1;
        if (!g_lines.empty()) sscanf(g_lines[0].c_str(), "Q,%lu,%d", &ms, &n);
        ck(n == 2, "an inverted window is read low-to-high, not as empty");

        // The recovery command a user reaches for when nothing works must undo
        // the one setting here that can stop a gun aiming.
        //
        // There are THREE formats now -- 0 basic, 1 extended, 2 full -- and
        // the wanted one still travels with its epoch in a single word.
        const int ep0 = wiicam_aim_fmt_epoch();
        wiicam_cam_command("cam=fmt:1");
        const int ep = wiicam_aim_fmt_epoch();
        ck(wiicam_aim_fmt() == WIICAM_FMT_EXT,
           "fmt:1 asks for the extended data format");
        ck(ep == ep0 + 1,
           "and bumps the epoch exactly ONCE so the camera owner re-applies it");
        wiicam_cam_command("cam=fmt:1");
        ck(wiicam_aim_fmt_epoch() == ep,
           "setting the SAME format again changes nothing -- every bump costs "
           "the camera poll a re-init with its settling delay");
        wiicam_cam_command("cam=fmt:2");
        ck(wiicam_aim_fmt() == WIICAM_FMT_FULL
           && wiicam_aim_fmt_epoch() == ep + 1,
           "fmt:2 asks for full mode, and that is one bump too, not two");
        wiicam_cam_command("cam=fmt:2");
        ck(wiicam_aim_fmt_epoch() == ep + 1,
           "full mode re-asked for is a no-op like the other two -- the third "
           "format must not be the one that re-inits the camera every poll");
        // 'ext' is the key name the tools used when there were only two
        // formats. It still works, and it still means the same numbers.
        wiicam_cam_command("cam=ext:1");
        ck(wiicam_aim_fmt() == WIICAM_FMT_EXT && wiicam_aim_fmt_epoch() == ep + 2,
           "the old 'ext' key still selects a format, so a tool that predates "
           "full mode is not silently ignored");
        // Clamping. A format this firmware does not have must land on one it
        // does -- the field is only two bits, so an unclamped 4 would arrive
        // as 0 and an unclamped 5 as 1.
        wiicam_cam_command("cam=fmt:9");
        ck(wiicam_aim_fmt() == WIICAM_FMT_FULL,
           "a format above the last one clamps to full rather than wrapping "
           "round the two-bit field");
        wiicam_cam_command("cam=fmt:-4");
        ck(wiicam_aim_fmt() == WIICAM_FMT_BASIC, "and a negative one to basic");

        wiicam_cam_command("cam=fmt:2");
        const int ep2 = wiicam_aim_fmt_epoch();
        wiicam_aim_format_dirty();
        ck(wiicam_aim_fmt_epoch() == ep2 + 1,
           "a camera restart bumps it: the format must be applied again or "
           "the frames decode as garbage with no error at all");
        ck(wiicam_aim_fmt() == WIICAM_FMT_FULL,
           "and the wanted format survives that bump");
        // The camera poll reads ONE word. Reading the format and the epoch as
        // two values let it pair a new epoch with the old format, write the
        // old format, latch the new epoch, and then never correct itself.
        //
        // The full-mode register byte joined them for exactly the same reason.
        // It is payload the poll acts on when the epoch moves, and a plain
        // variable stored beside a volatile one carries no ordering between
        // the two writes -- so the poll could re-init with the OLD byte while
        // cam? reported the new one, and the one knob that exists to rescue a
        // broken full mode would sit a version behind with nothing to correct
        // it. Three fields, one store, one read.
        ck((wiicam_aim_fmt_state() & 3) == wiicam_aim_fmt()
           && (wiicam_aim_fmt_state() >> 3) == wiicam_aim_fmt_epoch()
           && (((wiicam_aim_fmt_state() & 4) ? 0x05 : 0x55)
                   == wiicam_aim_fullreg()),
           "the state word carries all three, so no two of them can be read "
           "apart");
        // Every field this word has gained was gained by WIDENING it, and
        // every widening is a chance to leave a reader on the old shift --
        // which is the desync the single word exists to prevent. It has
        // happened twice now. Under (epoch << 1) | flag, full mode's 2 carried
        // straight out of a one-bit field and (epoch << 1) | 2 was
        // bit-identical to ((epoch + 1) << 1) | 0. Under (epoch << 2) | fmt,
        // selecting 0x05 would set bit 2 and read as an epoch one AHEAD with
        // the format unchanged: the poll re-initialising for an epoch that
        // never happened, and still writing the old byte.
        ck(wiicam_aim_fmt_state()
               == ((wiicam_aim_fmt_epoch() << 3)
                   | (wiicam_aim_fullreg() == 0x05 ? 4 : 0)
                   | WIICAM_FMT_FULL),
           "the word is (epoch << 3) | (freg05 << 2) | fmt, in that order");
        ck(wiicam_aim_fmt_state()
               != (((wiicam_aim_fmt_epoch() + 1) << 3) | WIICAM_FMT_BASIC),
           "so full mode at epoch N is a DIFFERENT word from basic at epoch "
           "N+1 -- with a one-bit format field those two aliased exactly");
        ck(((wiicam_aim_fmt_state() | 4) >> 3) == wiicam_aim_fmt_epoch(),
           "and setting the register bit cannot move the epoch: it has a bit "
           "of its own BELOW the epoch, not a place inside it");
        {
            const int st = wiicam_aim_fmt_state();
            wiicam_cam_command("cam=fmt:0");
            ck(wiicam_aim_fmt_state() != st,
               "and every change moves the whole word at once");
            ck(wiicam_aim_fmt() == WIICAM_FMT_BASIC
               && wiicam_aim_fmt_epoch() == (st >> 3) + 1
               && wiicam_aim_fullreg() == 0x55,
               "...all three fields landing from that one store, never "
               "separately, the register byte carried across untouched");
        }

        // ---- giving up on a format the sensor will not accept ---------------
        // An I2C master clocks out every byte it asks for whether or not the
        // sensor is producing them, so a 37-byte read against a sensor still
        // in extended returns 37 bytes, MATCHES on the retry -- the trailing
        // registers are static -- unpacks four plausible coordinates and
        // reports success. There is no error anywhere for a user to see, just
        // a cursor that is wrong until the next reboot. Dropping back is the
        // only outcome that is both safe and visible, because the tools watch
        // fmt and will show it fall on its own.
        wiicam_cam_command("cam=fmt:2");
        {
            const int epf = wiicam_aim_fmt_epoch();
            wiicam_aim_fmt_fallback(WIICAM_FMT_EXT);
            ck(wiicam_aim_fmt() == WIICAM_FMT_EXT
               && wiicam_aim_fmt_epoch() == epf + 1,
               "a fallback sets the format AND bumps the epoch, so the poll "
               "that asked for it really re-inits into what it fell back to");
            g_replies.clear();
            wiicam_cam_command("cam?");
            ck(!g_replies.empty()
               && g_replies[0].find("fmt=1") != std::string::npos,
               "and cam? shows the drop -- the only way anyone learns the "
               "sensor refused, since the write itself reports success");
            const int epf2 = wiicam_aim_fmt_epoch();
            wiicam_aim_fmt_fallback(WIICAM_FMT_EXT);
            ck(wiicam_aim_fmt_epoch() == epf2,
               "falling back to the format already in force costs nothing: "
               "the poll calls this on a refusal and refusals arrive in runs");
            wiicam_aim_fmt_fallback(99);
            ck(wiicam_aim_fmt() <= WIICAM_FMT_FULL,
               "an out-of-range fallback clamps to a format that exists "
               "rather than writing one no read length matches");
            wiicam_aim_fmt_fallback(WIICAM_FMT_BASIC);
            ck(wiicam_aim_fmt() == WIICAM_FMT_BASIC,
               "and it can go the whole way down to basic, the one format "
               "every sensor honours");
        }
        // cam? answers the old yes/no question -- "are sizes coming?" -- next
        // to the number, so a tool written for two formats is not left reading
        // a 2 as a boolean and getting it right by luck.
        wiicam_cam_command("cam=fmt:2");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("fmt=2") != std::string::npos
           && g_replies[0].find("ext=1") != std::string::npos,
           "cam? reports the format as a number and keeps ext as the boolean "
           "it always was");

        g_replies.clear();
        wiicam_cam_command("camreset");
        ck(wiicam_aim_fmt() == WIICAM_FMT_BASIC,
           "camreset turns the format back to basic -- from full as well as "
           "from extended");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty() && g_replies[0].find("bmin=0") != std::string::npos
           && g_replies[0].find("bmax=15") != std::string::npos,
           "...and opens the window again, so one command undoes all of it");

        // What the counters are FOR: the fraction of frames losing a corner.
        wiicam_cam_command("cam=res:2,dash:0");
        rig(px, py, 512, 384, 512, 288);
        for (int i = 0; i < 40; ++i) {       // lock on all four
            t += DT;
            px[1] += (i & 1) ? 1 : -1;
            wiicam_aim_process(px, py, 0xF, t, &sx, &sy);
        }
        g_replies.clear();
        wiicam_cam_command("camblob?");
        unsigned long r4a = 0;
        if (!g_replies.empty()) {
            const char* p4 = strstr(g_replies[0].c_str(), "br4=");
            if (p4) sscanf(p4, "br4=%lu", &r4a);
        }
        ck(r4a > 0, "frames with all four corners really seen are counted");
        for (int i = 0; i < 20; ++i) {       // now one LED is taken every frame
            t += DT;
            px[1] += (i & 1) ? 1 : -1;
            wiicam_aim_process(px, py, 0xF & ~(1u << 3), t, &sx, &sy);
        }
        g_replies.clear();
        wiicam_cam_command("camblob?");
        unsigned long r3 = 0;
        if (!g_replies.empty()) {
            const char* p3 = strstr(g_replies[0].c_str(), "br3=");
            if (p3) sscanf(p3, "br3=%lu", &r3);
        }
        ck(r3 >= 19, "and so are the frames that only got three -- the number "
                     "that says how much the light is costing");
    }

    // ---- the sensor's OWN thresholds (registers 0x06 / 0x1B) --------------
    // These gate inside the sensor, before it hands out its four slots, so
    // they are the only settings that can stop a stray source from COSTING us
    // a corner. Writing them takes settling time, so the write is a hook the
    // firmware calls from its serial-pump core, never from the camera poll.
    {
        wiicam_aim_begin();
        wiicam_set_blobreg_hook([](int reg, int val) -> int {
            if (g_reg_fail) return 0;          // a hook that cannot write yet
            g_reg.push_back(std::make_pair(reg, val));
            return 1;
        });
        // An earlier block left a camreset pending; drain it so this one starts
        // from "leave both registers alone".
        wiicam_cam_command("cam=hwmax:-1,hwmin:-1");
        wiicam_aim_hw_tick();
        g_reg.clear();
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("hwmax=-1") != std::string::npos
           && g_replies[0].find("hwmin=-1") != std::string::npos,
           "both sensor thresholds ship as 'leave the register alone'");
        g_reg.clear();
        wiicam_aim_hw_tick();
        ck(g_reg.empty(),
           "so a tick writes NOTHING -- the sensor is configured exactly as "
           "the build before this one");

        wiicam_cam_command("cam=hwmax:150");
        g_reg.clear();
        wiicam_aim_hw_tick();
        ck(g_reg.size() == 1 && g_reg[0].first == 0x06 && g_reg[0].second == 150,
           "hwmax reaches register 0x06, MAXSIZE");
        g_reg.clear();
        wiicam_aim_hw_tick();
        ck(g_reg.empty(), "and is not rewritten every loop");

        wiicam_cam_command("cam=hwmin:3");
        g_reg.clear();
        wiicam_aim_hw_tick();
        ck(g_reg.size() == 2 && g_reg[1].first == 0x1B && g_reg[1].second == 3,
           "hwmin reaches register 0x1B, MINSIZE -- which the stock driver "
           "has never written on any gun");

        // A camera rebuild re-runs the sensitivity preset, which rewrites
        // 0x06. Ours has to go back or it is silently lost.
        wiicam_aim_format_dirty();
        g_reg.clear();
        wiicam_aim_hw_tick();
        ck(g_reg.size() == 2,
           "a camera rebuild re-applies both: begin() rewrites 0x06 from the "
           "preset and would otherwise undo ours without a word");

        wiicam_cam_command("cam=hwmax:999,hwmin:-5");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("hwmax=255") != std::string::npos
           && g_replies[0].find("hwmin=-1") != std::string::npos,
           "values clamp to a byte, and any negative means leave it alone");
        g_reg.clear();
        wiicam_aim_hw_tick();
        ck(g_reg.size() == 1 && g_reg[0].first == 0x06,
           "so a 'leave alone' register really is not written");

        // camreset has to put the REGISTERS back, not merely stop tracking
        // them. Marking them "leave alone" left our value in the sensor, so
        // the one command a user reaches for when the gun has gone dark could
        // not undo the one setting able to make it go dark.
        wiicam_cam_command("cam=hwmax:40,hwmin:5");
        g_reg.clear(); wiicam_aim_hw_tick(); g_reg.clear();
        t_sens = 2;
        const int saves_before = t_sens;
        wiicam_cam_command("camreset");
        wiicam_aim_hw_tick();
        bool restored_min = false;
        for (size_t i = 0; i < g_reg.size(); ++i)
            if (g_reg[i].first == 0x1B && g_reg[i].second == 0) restored_min = true;
        ck(restored_min,
           "camreset writes MINSIZE back to 0 -- the direction that cannot "
           "cost an LED");
        ck(t_sens == saves_before,
           "and restores MAXSIZE by re-applying the sensitivity preset, which "
           "is the only value known to be sane for this sensor");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("hwmax=-1") != std::string::npos,
           "...after which it reports itself as leaving the register alone");

        // A refused write must keep the request pending, not consume it.
        g_reg_fail = true;
        wiicam_cam_command("cam=hwmax:44");
        wiicam_aim_hw_tick();
        g_reg_fail = false;
        g_reg.clear();
        wiicam_aim_hw_tick();
        ck(g_reg.size() == 1 && g_reg[0].second == 44,
           "a write refused because the camera was down is retried later, not "
           "forgotten while cam? claims it landed");

        // Zero in MAXSIZE is a dark gun, and it is where a typo lands.
        g_replies.clear();
        wiicam_cam_command("cam=hwmax:0");
        ck(!g_replies.empty()
           && g_replies[0].find("refused") != std::string::npos,
           "hwmax:0 is refused by name -- it tells the sensor to reject every "
           "blob");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty() && g_replies[0].find("hwmax=0") == std::string::npos,
           "and it does not take");

        // A key with no digits after the colon is a garbled line, not a zero.
        wiicam_cam_command("cam=hwmax:120");
        g_reg.clear(); wiicam_aim_hw_tick();
        wiicam_cam_command("cam=hwmax:");
        wiicam_cam_command("cam=hwmax:0x40");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty() && g_replies[0].find("hwmax=120") != std::string::npos,
           "a value-less or hex-looking value leaves the setting alone rather "
           "than reading as zero and blinding the sensor");
        wiicam_cam_command("camreset");
        wiicam_aim_hw_tick();
        wiicam_set_blobreg_hook(0);
    }

    // ---- the relative gate ------------------------------------------------
    // Four identical emitters driven identically should LOOK alike in one
    // frame. Judging by resemblance instead of by an absolute size is what
    // makes this immune to the thing that defeats fixed windows here: the play
    // distance varies about 2x, which is a 4x swing in brightness and so in
    // blob size. A fixed window must straddle all of it; this one need not.
    {
        wiicam_aim_begin();
        wiicam_cam_command("cam=res:0,dash:2,dashhz:0,mirx:1");
        rig(px, py, 512, 384, 512, 288);

        auto seen_n = [&](int sz[4]) {
            g_lines.clear();
            t += DT;
            px[0] += 1;                       // defeat the duplicate cache
            wiicam_aim_process_sz(px, py, sz, 0xF, t, &sx, &sy);
            int nn = -1; unsigned long mm;
            if (!g_lines.empty())
                sscanf(g_lines[0].c_str(), "Q,%lu,%d", &mm, &nn);
            return nn;
        };

        // The counters are cumulative since boot by design -- the tools show
        // deltas -- so the test reads them the same way.
        auto counters = [&](unsigned long* abs_rej, unsigned long* rel_rej) {
            g_replies.clear();
            wiicam_cam_command("camblob?");
            *abs_rej = *rel_rej = 0;
            if (g_replies.empty()) return;
            const char* a = strstr(g_replies[0].c_str(), "brej=");
            const char* r = strstr(g_replies[0].c_str(), "brrej=");
            if (a) sscanf(a, "brej=%lu", abs_rej);
            if (r) sscanf(r, "brrej=%lu", rel_rej);
        };

        int one_odd[4] = {3, 3, 3, 14};
        ck(seen_n(one_odd) == 4, "the relative gate ships OFF: nothing dropped");

        unsigned long a0, r0, a1, r1;
        counters(&a0, &r0);
        wiicam_cam_command("cam=rtol:3");   // steps, not percent
        ck(seen_n(one_odd) == 3,
           "with it on, the blob that does not match the other three is gone");
        counters(&a1, &r1);
        ck(r1 == r0 + 1 && a1 == a0,
           "and it is counted SEPARATELY from the absolute window, so the "
           "numbers say which gate did the work");

        // The floor. Three impostors outvote one real LED -- a consensus method
        // cannot do better, and the honest thing is to bound the damage rather
        // than pretend otherwise.
        int outvoted[4] = {3, 14, 14, 14};
        ck(seen_n(outvoted) == 3,
           "outvoted three to one it drops the odd blob out -- but never more "
           "than one, so three points always survive for the resolver");

        int spread[4] = {1, 5, 9, 15};
        ck(seen_n(spread) == 3,
           "even with no consensus at all the floor holds at three");

        int no_sizes[4] = {-1, -1, -1, -1};
        ck(seen_n(no_sizes) == 4,
           "and in basic format, with no sizes to compare, it cannot act");

        // Too few blobs to form a consensus at all.
        g_lines.clear();
        t += DT; px[0] += 1;
        wiicam_aim_process_sz(px, py, one_odd, 0x7, t, &sx, &sy);
        int n3 = -1;
        if (!g_lines.empty()) sscanf(g_lines[0].c_str(), "Q,%lu,%d", &ms, &n3);
        ck(n3 == 3, "with only three blobs seen it leaves them all alone");

        // It must not build its consensus out of blobs the absolute window
        // already refused -- that would be a consensus about the contamination.
        wiicam_cam_command("cam=bmin:0,bmax:15,rtol:0");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty() && g_replies[0].find("rtol=0") != std::string::npos,
           "rtol reports back, and 0 turns it off again");
    }

    // ---- full mode: the 37-byte report ------------------------------------
    // Every number in this format is arithmetic on bytes no header describes:
    // two coordinates whose high bits share ONE byte with the size, and four
    // box corners that are 7-bit inside 8-bit bytes. Nothing on the gun would
    // report any of it wrong -- a mis-shifted x is a plausible position and a
    // mis-masked box is a plausible box -- so it is checked here or nowhere.
    {
        wiicam_aim_begin();
        wiicam_cam_command("cam=res:0,dash:0,mirx:1,bmin:0,bmax:15,rtol:0");

        // Three real objects and one empty slot. The numbers are picked so a
        // shift applied to the wrong half of the shared byte cannot pass:
        // object 1 needs x's high bits from bits 4-5 and y's from bits 6-7 of
        // that same byte, and object 2's size fills the whole nibble.
        static const FullObj OBJ[4] = {
        //    x     y  sz   xmn   ymn   xmx  ymx  inten
            { 100,  200,  3,  10,   20, 0x92,  32, 200 },  // xmx = 18 | 0x80
            { 1000, 700,  7,   1, 0x82,    5,   9, 255 },  // ymn =  2 | 0x80
            { 513,  384, 15,  44,   50,   40,  53,   1 },  // xmx BELOW xmn
            { 1023,1023, 15,   0,    0,    0,   0,   0 },  // empty slot
        };
        memcpy(g_fobj, OBJ, sizeof(g_fobj));

        int fpx[4], fpy[4], fsz[4];
        unsigned fseen = 0xDEADu;
        for (int i = 0; i < 4; ++i) { fpx[i] = fpy[i] = fsz[i] = -777; }

        // No hook installed is the boot state, and every build that never
        // asks for full mode stays in it.
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 0,
           "with no bus hook there is no frame");
        ck(fseen == 0xDEADu && fpx[0] == -777,
           "...and nothing is written -- a seen mask left as whatever was on "
           "the stack is four phantom corners");

        wiicam_set_fullread_hook(full_hook);
        g_fcalls = 0; g_flen = 0; g_ffail_on = 0; g_fdrift = 0; g_fhdrdrift = 0;
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 1,
           "a steady sensor reads clean");
        ck(g_flen == 37,
           "and the read is 37 bytes -- 1 header + 4 objects x 9. A length "
           "that does not match the format register still returns bytes and "
           "still reports success, it just decodes to nonsense");
        ck(g_fcalls == 2,
           "read TWICE and compared: the sensor has no frame sync, so a "
           "report can be read across an update and come back half old");

        ck(fpx[0] == 100 && fpy[0] == 200 && fsz[0] == 3,
           "object 0 unpacks to x, y and the 4-bit size");
        ck(fpx[1] == 1000 && fpy[1] == 700 && fsz[1] == 7,
           "so does object 1, whose x needs bits 4-5 of the shared byte and "
           "whose y needs bits 6-7 -- swap the two shifts and both are still "
           "plausible positions");
        ck(fpx[2] == 513 && fpy[2] == 384 && fsz[2] == 15,
           "and object 2, whose size fills the nibble the coordinates share");
        ck(fseen == 0x7u,
           "the empty slot reports y = 1023, above 767, which is the driver's "
           "'not seen' -- so it is absent from the mask");
        ck(fpx[3] == -777 && fpy[3] == -777 && fsz[3] == -777,
           "and its slot is left ALONE: a 1023,1023 written through is a real "
           "corner to anything downstream that forgets the mask");

        // 767 is the last row the sensor can report and 768 the first value
        // that means nothing is there. Off by one here either throws away a
        // real bottom-edge corner or admits a phantom one at the floor.
        g_fobj[3].y = 767;
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 1 && (fseen & 8u),
           "y == 767 is a real object at the very bottom of the frame");
        g_fobj[3].y = 768;
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 1 && !(fseen & 8u),
           "y == 768 is the first value that means the slot is empty");
        g_fobj[3].y = 1023;

        // Only the HEADER byte differs between the two reads. That is not a
        // torn frame -- byte 0 belongs to no object -- and a memcmp over all
        // 37 bytes would call it one on every single poll.
        g_fcalls = 0; g_fhdrdrift = 1;
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 1 && g_fcalls == 2,
           "a header byte that changes between the two reads is NOT a "
           "mismatch: the compare starts at byte 1");
        g_fhdrdrift = 0;

        // A frame that never repeats is DROPPED. This used to publish the
        // newest read instead, on the reasoning that it matched the driver's
        // Retry_1s -- but GetPosition asks for Retry_2, and the driver's EVEN
        // retry counts return Error_DataMismatch, so both formats this stands
        // beside drop the frame too. Publishing it made full mode the one path
        // that would hand the resolver a torn report, in precisely the case
        // the double read exists to catch: half a frame from before a fast pan
        // and half from after is a quad that never existed anywhere, and the
        // cursor jumps to a place nothing was ever aimed at. One lost frame is
        // 5 ms at 200 Hz; the resolver coasts through it without noticing.
        for (int i = 0; i < 4; ++i) { fpx[i] = fpy[i] = fsz[i] = -777; }
        fseen = 0xDEADu;
        g_fcalls = 0; g_fdrift = 1;
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 0,
           "a frame that differs on every read is dropped, not published -- "
           "full mode must not be more permissive than the two formats it "
           "stands in for");
        ck(g_fcalls == 3,
           "and it gives up after ONE retry -- three reads, never a spin");
        ck(fseen == 0xDEADu && fpx[0] == -777 && fsz[0] == -777,
           "a dropped frame writes nothing, exactly like a bus error: the "
           "caller's arrays still hold the last frame that was whole");
        g_fdrift = 0;

        // A bus error. The caller's arrays still hold the previous frame, and
        // a half-written one is worse than none: the resolver cannot tell.
        for (int i = 0; i < 4; ++i) { fpx[i] = fpy[i] = fsz[i] = -777; }
        fseen = 0xDEADu;
        g_fcalls = 0; g_ffail_on = 1;
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 0,
           "a read that fails outright is reported as no frame");
        ck(fseen == 0xDEADu && fpx[0] == -777 && fsz[0] == -777,
           "and writes nothing at all -- not even the seen mask");
        // The SECOND read is the easy one to get wrong: by then the first
        // buffer is full of good-looking bytes and returning them looks free.
        // It is not -- that frame was never confirmed against anything.
        g_fcalls = 0; g_ffail_on = 2;
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 0,
           "a bus that dies BETWEEN the two reads is no frame either");
        ck(fseen == 0xDEADu && fpx[0] == -777,
           "...and still writes nothing");
        g_ffail_on = 0;

        // The box, the intensity and the box ORIGIN are REPORTED, not gated: a
        // discriminator invented before anyone has seen a real number out of
        // it is the same guess the size window would have been. So
        // '~camblob?' is the only way they reach anyone, and in full mode each
        // tuple grows by five.
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 1, "a clean frame again");
        wiicam_cam_command("cam=fmt:2");
        t += DT;
        wiicam_aim_process_sz(fpx, fpy, fsz, fseen, t, &sx, &sy);
        g_replies.clear();
        wiicam_cam_command("camblob?");
        int f[27];
        int got = 0;
        if (g_replies.size() >= 2)
            got = sscanf(g_replies[1].c_str(),
                         "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                         " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                         " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                         &f[0],&f[1],&f[2],&f[3],&f[4],&f[5],&f[6],&f[7],&f[8],
                         &f[9],&f[10],&f[11],&f[12],&f[13],&f[14],&f[15],&f[16],&f[17],
                         &f[18],&f[19],&f[20],&f[21],&f[22],&f[23],&f[24],&f[25],&f[26]);
        ck(got == 27,
           "in full mode every blob tuple carries NINE fields: the four the "
           "other formats have, plus box width, box height, intensity and the "
           "box origin's two coordinates");
        ck(got == 27 && f[2] == 3 && f[3] == 1,
           "the first four are still position, size and the gate's verdict, "
           "in that order -- the extras are appended, not inserted");
        ck(got == 27 && f[4] == 8 && f[5] == 12 && f[6] == 200,
           "object 0's box is 8x12 at intensity 200: its xMax arrived as 0x92 "
           "and the corners are SEVEN bit, so read as eight it would be a "
           "136-wide box on a blob three pixels across");
        ck(got == 27 && f[13] == 4 && f[14] == 7 && f[15] == 255,
           "object 1's yMin arrived as 0x82; masked to 2 it still gives a "
           "positive height instead of collapsing to zero");
        ck(got == 27 && f[22] == 0 && f[23] == 3 && f[24] == 1,
           "and a box whose xMax is BELOW its xMin reads as zero width, not "
           "as the 252 an unsigned subtraction wraps to");
        // The ORIGIN is the same two bytes the width and the height are
        // derived FROM, published raw. It is what settles what the box fields
        // actually mean -- centre against reported position decides whether
        // they are 7-bit native or something else -- so it has to arrive
        // exactly as the sensor sent it, masked and not otherwise touched.
        ck(got == 27 && f[7] == 10 && f[8] == 20,
           "object 0's origin is its xMin,yMin, in that order -- 10,20 and not "
           "20,10, which a transposed pair would be indistinguishable from on "
           "a square box");
        ck(got == 27 && f[16] == 1 && f[17] == 2,
           "object 1's yMin arrived as 0x82 and is published as 2: the origin "
           "is masked to seven bits like the corners it comes from, or the box "
           "the report draws sits 128 rows down the frame from the blob");
        ck(got == 27 && f[25] == 44 && f[26] == 50,
           "and object 2 keeps its origin even though its xMax is BELOW its "
           "xMin -- the width collapses to zero, the corner the sensor sent "
           "does not");

        // The same nine columns again on a frame where NO TWO FIELDS SHARE A
        // VALUE, keep flags aside. The origin is two bytes of a nine-byte
        // object, sitting immediately before the two corners it is subtracted
        // from to make the width and the height -- so an index one byte off,
        // a transposed pair, or a corner published in place of the origin are
        // all arithmetic slips that produce a perfectly plausible box in a
        // perfectly plausible place. Every one of them lands on a number that
        // belongs to another column, and with all twenty-four distinct there
        // is no column a wrong read can hide in.
        static const FullObj UNIQ[4] = {
        //     x     y  sz   xmn   ymn  xmx  ymx  inten
            {  100, 200,  3,   71,   62,  80,  76, 181 },  // box  9x14 at 71,62
            {  400, 300,  5,   88,   97, 110, 127, 203 },  // box 22x30 at 88,97
            {  700, 500, 11, 0xC1, 0x93,  72,  53, 250 },  // box  7x34 at 65,19
            { 1023,1023, 15,    0,    0,   0,   0,   0 },  // empty
        };
        memcpy(g_fobj, UNIQ, sizeof(g_fobj));
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 1 && fseen == 0x7u,
           "three more objects, and no number in the frame appears twice");
        t += DT;
        wiicam_aim_process_sz(fpx, fpy, fsz, fseen, t, &sx, &sy);
        g_replies.clear();
        wiicam_cam_command("camblob?");
        int u[27];
        int gotu = 0;
        if (g_replies.size() >= 2)
            gotu = sscanf(g_replies[1].c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                          &u[0],&u[1],&u[2],&u[3],&u[4],&u[5],&u[6],&u[7],&u[8],
                          &u[9],&u[10],&u[11],&u[12],&u[13],&u[14],&u[15],&u[16],&u[17],
                          &u[18],&u[19],&u[20],&u[21],&u[22],&u[23],&u[24],&u[25],&u[26]);
        ck(gotu == 27
           && u[0] == 216 && u[1] == 46 && u[2] == 3 && u[3] == 1
           && u[4] == 9 && u[5] == 14 && u[6] == 181
           && u[7] == 71 && u[8] == 62,
           "blob 0 reads 216,46,3,1,9,14,181,71,62 -- nine fields, the origin "
           "the last two of them, xMin before yMin");
        ck(gotu == 27
           && u[9] == 146 && u[10] == 69 && u[11] == 5 && u[12] == 1
           && u[13] == 22 && u[14] == 30 && u[15] == 203
           && u[16] == 88 && u[17] == 97,
           "blob 1 reads 146,69,5,1,22,30,203,88,97 -- its yMax is 127, the "
           "largest a 7-bit corner holds, and the origin under it is still 97 "
           "and not the 30 it was subtracted to make");
        ck(gotu == 27
           && u[18] == 76 && u[19] == 115 && u[20] == 11 && u[21] == 1
           && u[22] == 7 && u[23] == 34 && u[24] == 250
           && u[25] == 65 && u[26] == 19,
           "and blob 2 reads 76,115,11,1,7,34,250,65,19 -- both origin bytes "
           "arrived with the top bit set, 0xC1 and 0x93, and both come out "
           "masked to seven bits rather than as 193 and 147");

        // Both reply lines come from ONE read of the count. Read separately,
        // the count came from one frame and the list from the next: 1.4% of
        // the rows in the first real capture disagreed with themselves.
        g_replies.clear();
        wiicam_cam_command("camblob?");
        unsigned long bnv = 99;
        int tuples = 0;
        if (g_replies.size() >= 2) {
            const char* pb = strstr(g_replies[0].c_str(), "bn=");
            if (pb) sscanf(pb, "bn=%lu", &bnv);
            const std::string& bl = g_replies[1];
            const size_t at = bl.find("blobs");
            if (at != std::string::npos)
                for (size_t i = at; i + 1 < bl.size(); ++i)
                    if (bl[i] == ' '
                        && (bl[i+1] == '-' || (bl[i+1] >= '0' && bl[i+1] <= '9')))
                        ++tuples;
        }
        ck(bnv == 3 && tuples == 3,
           "the count and the list are taken from ONE snapshot, so the two "
           "lines always describe the same frame");

        // Outside full mode the tuple must stay four fields wide, or a tool
        // written for extended starts reading the next blob's x as this
        // blob's box width and every column after it shifts.
        wiicam_cam_command("cam=fmt:1");
        g_replies.clear();
        wiicam_cam_command("camblob?");
        ck(g_replies.size() >= 2
           && sscanf(g_replies[1].c_str(), "CAM: blobs %d,%d,%d,%d,%d",
                     &f[0],&f[1],&f[2],&f[3],&f[4]) == 4,
           "in extended format the tuple is four fields and stops there");
        wiicam_cam_command("cam=fmt:0");
        g_replies.clear();
        wiicam_cam_command("camblob?");
        ck(g_replies.size() >= 2
           && g_replies[1].find("(sizes need fmt:1)") != std::string::npos,
           "and in basic format it says why there are no sizes, naming the "
           "key that turns them on");

        // ---- the box and its origin must travel with their own blob --------
        // The poll fills its box arrays by HARDWARE SLOT, 0..3 as the sensor
        // numbers them. Everything else in the report -- position, size, kept
        // -- is indexed by the blob's place in the COMPACTED seen list,
        // because process_sz skips empty slots as it walks. Publish the box
        // straight from the slot-indexed array and the two agree only while
        // the empty slot is the LAST one: any other gap and blob 0 is printed
        // with slot 0's box, which belongs to no blob on screen and may be
        // left over from an earlier frame entirely.
        //
        // The ORIGIN arrives through the same two arrays and the same
        // compaction, so it can go wrong in exactly the same way and has to be
        // checked in exactly the same place. Every fixture below therefore
        // gives each slot its OWN origin as well as its own box: with all four
        // sharing one origin the columns would agree no matter which index
        // they were read by, and the check would be measuring nothing.
        //
        // Which is the whole point of the instrumentation: a slot goes empty
        // exactly when a bright window has taken it, so the readout would
        // have been wrong precisely in the case it exists to measure. The
        // tests above all use a TRAILING empty slot, the one arrangement in
        // which the two indexings agree, so the gaps get their own block.
        g_ffail_on = 0; g_fdrift = 0; g_fhdrdrift = 0;
        wiicam_cam_command("cam=res:0,dash:0,mirx:1,bmin:0,bmax:15,rtol:0");
        wiicam_cam_command("cam=fmt:2");

        // The arrays live out here so an unseen slot keeps its previous
        // contents between frames -- which is what the driver does, and what
        // the mask exists to protect the pipeline from.
        int qpx[4] = {0, 0, 0, 0}, qpy[4] = {0, 0, 0, 0};
        int qsz[4] = {-1, -1, -1, -1};
        unsigned qseen = 0;
        // One full-mode frame end to end: fill the slots, poll, and hand the
        // result to the pipeline the way the firmware does. 'mask' is what
        // process_sz is TOLD was seen -- normally the poll's own answer.
        auto full_report = [&](const FullObj* o, unsigned mask) {
            memcpy(g_fobj, o, sizeof(g_fobj));
            wiicam_aim_full_poll(qpx, qpy, qsz, &qseen);
            t += DT;
            wiicam_aim_process_sz(qpx, qpy, qsz, mask, t, &sx, &sy);
            g_replies.clear();
            wiicam_cam_command("camblob?");
            return g_replies.size() >= 2 ? g_replies[1] : std::string();
        };
        int fb[36];
        int gotb = 0;

        // Slot ZERO empty. Every box, origin and intensity below is distinct,
        // so a tuple wearing the wrong one says which slot it was taken from.
        static const FullObj GAP0[4] = {
        //    x     y  sz  xmn  ymn  xmx  ymx  inten
            { 1023,1023, 15,  0,   0,   0,   0,   0 },  // EMPTY
            {  100, 200,  3, 10,  20,  18,  32, 111 },  // box  8x12 at 10,20
            {  300, 250,  4, 31,  42,  51,  63, 122 },  // box 20x21 at 31,42
            {  500, 300,  5, 53,  64,  93, 105, 133 },  // box 40x41 at 53,64
        };
        {
            const std::string ln = full_report(GAP0, 0xEu);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],&fb[7],&fb[8],
                          &fb[9],&fb[10],&fb[11],&fb[12],&fb[13],&fb[14],&fb[15],&fb[16],&fb[17],
                          &fb[18],&fb[19],&fb[20],&fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26]);
            ck(qseen == 0xEu && gotb == 27,
               "slot 0 empty: the report lists the three blobs that are there");
            ck(gotb == 27 && fb[2] == 3 && fb[4] == 8 && fb[5] == 12
               && fb[6] == 111,
               "and the FIRST tuple carries slot 1's box, because slot 1 is "
               "the first blob -- slot 0's box belongs to no blob on screen");
            ck(gotb == 27 && fb[11] == 4 && fb[13] == 20 && fb[14] == 21
               && fb[15] == 122,
               "the second carries slot 2's, not slot 1's -- every entry after "
               "a gap is off by one the moment the box is read by slot");
            ck(gotb == 27 && fb[20] == 5 && fb[22] == 40 && fb[23] == 41
               && fb[24] == 133,
               "and the third slot 3's, so the last real blob's box is printed "
               "at all rather than falling off the end of the list");
            // The origin, through the same compaction. A LEADING empty slot is
            // the arrangement the earlier slot/index bug hid in and the one it
            // shipped in: read by the compacted index the first tuple gets
            // slot 0's origin, which the poll cleared to 0,0 -- a box drawn at
            // the top-left corner of the sensor for a blob that is nowhere
            // near it, and 0,0 is exactly the value that reads as plausible.
            ck(gotb == 27 && fb[7] == 10 && fb[8] == 20,
               "the first tuple's origin is slot 1's 10,20 and not the empty "
               "slot 0's cleared 0,0");
            ck(gotb == 27 && fb[16] == 31 && fb[17] == 42
               && fb[25] == 53 && fb[26] == 64,
               "and the other two carry slot 2's and slot 3's -- the origin is "
               "compacted through the same slot map as the box it belongs to, "
               "not published one blob out of step with it");
        }

        // A gap in the MIDDLE shifts only the tail, so the first entries look
        // right and only the ones after the gap are wrong -- the version of
        // this bug that survives a glance at the readout.
        static const FullObj GAP2[4] = {
        //    x     y  sz  xmn  ymn  xmx  ymx  inten
            {  110, 210,  6,  1,   2,  10,  15, 144 },  // box  9x13 at  1,2
            {  310, 260,  7, 17,  23,  39,  46, 155 },  // box 22x23 at 17,23
            { 1023,1023, 15,  0,   0,   0,   0,   0 },  // EMPTY
            {  510, 310,  8, 35,  41,  79,  86, 166 },  // box 44x45 at 35,41
        };
        {
            const std::string ln = full_report(GAP2, 0xBu);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],&fb[7],&fb[8],
                          &fb[9],&fb[10],&fb[11],&fb[12],&fb[13],&fb[14],&fb[15],&fb[16],&fb[17],
                          &fb[18],&fb[19],&fb[20],&fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26]);
            ck(qseen == 0xBu && gotb == 27, "slot 2 empty: three blobs again");
            ck(gotb == 27 && fb[4] == 9 && fb[5] == 13 && fb[6] == 144
               && fb[13] == 22 && fb[14] == 23 && fb[15] == 155,
               "the two blobs BEFORE the gap keep their own boxes -- this is "
               "the shape of the bug that reads as fine until the tail");
            ck(gotb == 27 && fb[20] == 8 && fb[22] == 44 && fb[23] == 45
               && fb[24] == 166,
               "and the one after it carries slot 3's box, not the empty "
               "slot 2's zeroes");
            ck(gotb == 27 && fb[7] == 1 && fb[8] == 2
               && fb[16] == 17 && fb[17] == 23 && fb[25] == 35 && fb[26] == 41,
               "and the origins follow the same three blobs, the one past the "
               "gap included -- a mid-frame gap shifts only the tail, which is "
               "the version of this that survives a glance at the readout");
        }

        // Leaving full mode. The box and origin columns stop being printed,
        // and there is nowhere for the last full frame's numbers to hide in a
        // four-field tuple -- so count the commas rather than trusting the
        // first one.
        wiicam_cam_command("cam=fmt:1");
        g_replies.clear();
        wiicam_cam_command("camblob?");
        {
            int commas = 0;
            if (g_replies.size() >= 2) {
                const std::string& ln = g_replies[1];
                const size_t at = ln.find("blobs");
                if (at != std::string::npos)
                    for (size_t i = at; i < ln.size(); ++i)
                        if (ln[i] == ',') ++commas;
            }
            ck(commas == 9,
               "back in extended the three tuples carry four fields each and "
               "nothing more: nine commas, not the twenty-four a full-mode "
               "line of the same three blobs would carry");
        }
        // And going back to full mode must not resurrect them. The moment the
        // format is 2 again the box columns reappear, and until a full frame
        // has actually been read there is nothing to put in them -- the last
        // full frame's boxes attached to whatever blobs are current is a
        // measurement of two different moments printed as one.
        wiicam_cam_command("cam=fmt:2");
        t += DT;
        wiicam_aim_process_sz(qpx, qpy, qsz, 0xBu, t, &sx, &sy);
        g_replies.clear();
        wiicam_cam_command("camblob?");
        gotb = 0;
        if (g_replies.size() >= 2)
            gotb = sscanf(g_replies[1].c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],&fb[7],&fb[8],
                          &fb[9],&fb[10],&fb[11],&fb[12],&fb[13],&fb[14],&fb[15],&fb[16],&fb[17],
                          &fb[18],&fb[19],&fb[20],&fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26]);
        ck(gotb == 27 && fb[4] == 0 && fb[5] == 0 && fb[6] == 0
           && fb[13] == 0 && fb[14] == 0 && fb[15] == 0
           && fb[22] == 0 && fb[23] == 0 && fb[24] == 0,
           "a format round trip through extended leaves every box at zero "
           "until a full frame is read again");
        // The ORIGIN arrays are cleared by the same leave-full-mode path, and
        // they have to be: 1,2 and 17,23 left behind from the last full frame
        // are a box drawn at a real place on the sensor for blobs measured in
        // a format that never sent one, which is a more convincing lie than a
        // stale width. Zero at least admits it knows nothing.
        ck(gotb == 27 && fb[7] == 0 && fb[8] == 0
           && fb[16] == 0 && fb[17] == 0 && fb[25] == 0 && fb[26] == 0,
           "...and every origin with them, rather than the last full frame's "
           "corners surviving the trip out of full mode and back");

        // An unseen slot reads as NO BOX, never as the box it had last frame.
        static const FullObj FOUR[4] = {
        //    x     y  sz  xmn  ymn  xmx  ymx  inten
            { 120, 220,  2,  1,   2,  12,  15,  77 },   // box 11x13 at  1,2
            { 320, 270,  6, 14,  19,  40,  52,  88 },   // box 26x33 at 14,19
            { 520, 320,  9, 26,  31,  86,  95,  99 },   // box 60x64 at 26,31
            { 720, 370, 11, 37,  43, 107, 117, 210 },   // box 70x74 at 37,43
        };
        static const FullObj THREE[4] = {
            { 120, 220,  2,  1,   2,  12,  15,  77 },
            { 320, 270,  6, 14,  19,  40,  52,  88 },
            { 520, 320,  9, 26,  31,  86,  95,  99 },
            { 1023,1023, 15, 0,   0,   0,   0,   0 },   // slot 3 goes dark
        };
        {
            const std::string ln = full_report(FOUR, 0xFu);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],&fb[7],&fb[8],
                          &fb[9],&fb[10],&fb[11],&fb[12],&fb[13],&fb[14],&fb[15],&fb[16],&fb[17],
                          &fb[18],&fb[19],&fb[20],&fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26],
                          &fb[27],&fb[28],&fb[29],&fb[30],&fb[31],&fb[32],&fb[33],&fb[34],&fb[35]);
            ck(qseen == 0xFu && gotb == 36,
               "with no gap at all, four tuples and four boxes");
            ck(gotb == 36 && fb[4] == 11 && fb[13] == 26 && fb[22] == 60
               && fb[31] == 70,
               "each the box of the blob it is printed beside -- the case "
               "where slot order and report order happen to be the same");
            ck(gotb == 36 && fb[7] == 1 && fb[16] == 14 && fb[25] == 26
               && fb[34] == 37,
               "...and each its own origin, so the one arrangement where the "
               "two indexings agree is on record as agreeing rather than "
               "merely never having been looked at");
        }
        {
            // Same three blobs, slot 3 dark. Nothing of slot 3's 70x74 at 210,
            // origin 37,43, may appear anywhere in this frame's report.
            const std::string ln = full_report(THREE, 0x7u);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d %d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],&fb[7],&fb[8],
                          &fb[9],&fb[10],&fb[11],&fb[12],&fb[13],&fb[14],&fb[15],&fb[16],&fb[17],
                          &fb[18],&fb[19],&fb[20],&fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26],
                          &fb[27]);
            ck(qseen == 0x7u && gotb == 27,
               "the slot that went dark is dropped from the list, and no "
               "fourth tuple is printed from the frame before it");
            ck(gotb == 27 && fb[4] == 11 && fb[13] == 26 && fb[22] == 60,
               "the three that remain still wear their own boxes");
        }
        {
            // The stale-slot trap, in full mode. The poll leaves an unseen
            // slot's coordinates alone, so a caller that forgets the mask
            // hands the pipeline slot 3 as though it were still there. Its
            // box must read as nothing measured -- last frame's 70x74 at 210
            // printed beside last frame's stale position is two moments in
            // one row, and nothing downstream could tell.
            const std::string ln = full_report(THREE, 0xFu);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],&fb[7],&fb[8],
                          &fb[9],&fb[10],&fb[11],&fb[12],&fb[13],&fb[14],&fb[15],&fb[16],&fb[17],
                          &fb[18],&fb[19],&fb[20],&fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26],
                          &fb[27],&fb[28],&fb[29],&fb[30],&fb[31],&fb[32],&fb[33],&fb[34],&fb[35]);
            ck(gotb == 36 && fb[31] == 0 && fb[32] == 0 && fb[33] == 0,
               "an unseen slot reads as no box at all -- 0x0 at intensity 0 "
               "-- rather than as the box it measured on the frame before");
            ck(gotb == 36 && fb[34] == 0 && fb[35] == 0,
               "...and as no origin either: last frame's 37,43 printed beside "
               "this frame's stale position is two moments in one row, and a "
               "corner that really is on the sensor reads as more trustworthy "
               "than a stale width does");
            ck(gotb == 36 && fb[4] == 11 && fb[13] == 26 && fb[22] == 60,
               "and the blobs that ARE there are untouched by that");
        }

        // ---- the line at its WIDEST -----------------------------------------
        // Four blobs, every field as many characters as the sensor can make
        // it: a position at the far edge of 240x176 space, a size filling the
        // nibble, a box origin up in three figures with a two-figure side
        // beside it -- the two trade off inside the 7-bit corner, so this is
        // the widest a real box gets -- and the intensity byte at 255.
        //
        // The tuple went from four fields to seven and then to nine, and each
        // time the per-blob RESERVE the loop keeps back had to grow with it.
        // Get that reserve wrong, or leave the buffer where it was, and the
        // loop stops early: the fourth blob's tuple is simply not printed, the
        // line is still well-formed, and every log drawn from it is quietly
        // short a row in exactly the frames where all four corners were seen.
        static const FullObj WIDEST[4] = {
        //    x    y  sz  xmn  ymn  xmx  ymx  inten
            {   0, 700, 15, 100, 104, 127, 127, 255 },   // box 27x23 at 100,104
            { 100, 730, 15, 101, 105, 127, 127, 255 },   // box 26x22 at 101,105
            { 200, 750, 15, 102, 106, 127, 127, 255 },   // box 25x21 at 102,106
            { 300, 760, 15, 103, 107, 127, 127, 255 },   // box 24x20 at 103,107
        };
        {
            const std::string ln = full_report(WIDEST, 0xFu);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],&fb[7],&fb[8],
                          &fb[9],&fb[10],&fb[11],&fb[12],&fb[13],&fb[14],&fb[15],&fb[16],&fb[17],
                          &fb[18],&fb[19],&fb[20],&fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26],
                          &fb[27],&fb[28],&fb[29],&fb[30],&fb[31],&fb[32],&fb[33],&fb[34],&fb[35]);
            ck(gotb == 36,
               "four maximum-width blobs still print all four nine-field "
               "tuples: 36 numbers, none of them lost to the buffer");
            ck(gotb == 36 && fb[27] == 169 && fb[28] == 174 && fb[29] == 15
               && fb[30] == 1 && fb[31] == 24 && fb[32] == 20 && fb[33] == 255
               && fb[34] == 103 && fb[35] == 107,
               "...and the LAST tuple is complete down to its final ymn, which "
               "is the character a line one byte too long loses first");
            ck(!ln.empty() && ln[ln.size() - 1] == '\n',
               "...and the line still ends where it should: a reply cut off by "
               "its own buffer loses the newline before it loses anything a "
               "parser would notice");
        }
        {
            // Line 1 grew with the same change -- three more gate keys since
            // it was last sized -- and it is the longer of the two by far.
            // Its last field is the one a short buffer eats.
            g_replies.clear();
            wiicam_cam_command("camblob?");
            const std::string l1 = g_replies.empty() ? std::string() : g_replies[0];
            ck(l1.find("br0=") != std::string::npos
               && !l1.empty() && l1[l1.size() - 1] == '\n',
               "the counter line reaches br0, its last field, and ends in a "
               "newline -- with bhmax added it is the longest reply the '~cam' "
               "surface emits, and a truncated one drops the frame buckets the "
               "percentages are computed from");
        }
        // The two reply lines go out back to back, and the camera poll does
        // not stop between them. Snapshotting the COUNT stopped the two lines
        // disagreeing about how many blobs there were; it did nothing about
        // WHICH, so a frame landing in between left line 2 printing this
        // frame's first blobs and the previous frame's last ones under a count
        // that looked perfectly consistent. Both are copied now. Simulated by
        // publishing a frame from inside the reply sink, which is exactly
        // where the other core would land.
        {
            full_report(GAP0, 0xEu);            // frame A: three blobs
            for (int i = 0; i < 4; ++i) {       // frame B: four, all different
                g_race_px[i] = 200 + i * 60;
                g_race_py[i] = 300 + i * 20;
                g_race_sz[i] = 9;
            }
            g_race_seen = 0xFu;
            t += DT;
            g_race_t = t;
            g_race_arm = true;
            wiicam_set_reply_sink(reply_sink_racing);
            g_replies.clear();
            wiicam_cam_command("camblob?");
            wiicam_set_reply_sink(reply_sink);
            ck(!g_race_arm,
               "the racing frame really did land between the two lines");
            unsigned long rbn = 99;
            gotb = 0;
            if (g_replies.size() >= 2) {
                const char* pb = strstr(g_replies[0].c_str(), "bn=");
                if (pb) sscanf(pb, "bn=%lu", &rbn);
                gotb = sscanf(g_replies[1].c_str(),
                              "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d,%d,%d %d",
                              &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],&fb[7],&fb[8],
                              &fb[9],&fb[10],&fb[11],&fb[12],&fb[13],&fb[14],&fb[15],&fb[16],&fb[17],
                              &fb[18],&fb[19],&fb[20],&fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26],
                              &fb[27]);
            }
            ck(rbn == 3 && gotb == 27,
               "a frame landing mid-reply changes neither line: still three "
               "counted and three listed, not four");
            ck(gotb == 27 && fb[2] == 3 && fb[4] == 8 && fb[7] == 10
               && fb[11] == 4 && fb[13] == 20 && fb[16] == 31
               && fb[20] == 5 && fb[22] == 40 && fb[25] == 53,
               "and they are the SAME three -- sizes, boxes and origins from "
               "the frame line 1 counted, not the one that arrived between the "
               "lines");
        }
        wiicam_cam_command("cam=fmt:0");

        // ---- the mode byte -------------------------------------------------
        // We are honestly not certain of it. The driver's own WORKING
        // constants for the other two formats are the doubled nibble -- 0x11
        // basic, 0x33 extended -- so 0x55 is what this sensor already accepts
        // every power-on; wiibrew documents the modes as 1/3/5, under which
        // 0x05 works too. Both readings are defensible, so the byte is
        // settable -- but only between those two, because anything else
        // leaves the sensor in a state no read length matches and nothing
        // anywhere reports an error.
        ck(wiicam_aim_fullreg() == 0x55,
           "full mode ships on 0x55, the value that is right under BOTH "
           "readings where 0x05 is right under only one");
        wiicam_cam_command("cam=fullreg:5");
        ck(wiicam_aim_fullreg() == 0x05,
           "fullreg:5 tries wiibrew's low-nibble reading without a reflash");
        wiicam_cam_command("cam=fullreg:85");
        ck(wiicam_aim_fullreg() == 0x55, "and fullreg:85 goes back");
        g_replies.clear();
        wiicam_cam_command("cam=fullreg:55");
        ck(wiicam_aim_fullreg() == 0x55,
           "55 -- what a user typing the hex 0x55 as decimal lands on -- does "
           "not take");
        ck(!g_replies.empty()
           && g_replies[0].find("must be 5 or 85") != std::string::npos,
           "...and is refused BY NAME, so the typo is visible instead of "
           "living on in the format register");
        wiicam_cam_command("cam=fullreg:0");
        wiicam_cam_command("cam=fullreg:255");
        wiicam_cam_command("cam=fullreg:-5");
        ck(wiicam_aim_fullreg() == 0x55, "nor do 0, 255 or a negative");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("fullreg=85") != std::string::npos,
           "cam? reports the byte in force, so a bench session can say which "
           "of the two readings was actually tried");
        // Changing it while full mode is LIVE has to re-apply the format, or
        // the new byte never reaches the register it exists for -- and the
        // byte and the epoch have to land in ONE store, or a poll that has
        // seen the new epoch can still be holding the old byte.
        wiicam_cam_command("cam=fmt:2");
        const int epr = wiicam_aim_fmt_epoch();
        const int str = wiicam_aim_fmt_state();
        wiicam_cam_command("cam=fullreg:5");
        ck(wiicam_aim_fmt_epoch() == epr + 1
           && wiicam_aim_fmt() == WIICAM_FMT_FULL
           && wiicam_aim_fullreg() == 0x05,
           "changing it in full mode re-applies the format so the new byte "
           "actually gets written");
        ck(wiicam_aim_fmt_state() != str
           && wiicam_aim_fmt_state()
                  == (((epr + 1) << 3) | 4 | WIICAM_FMT_FULL),
           "and the byte, the format and the epoch move together in one word "
           "-- the poll cannot re-init for the new epoch with the old byte");
        // Outside full mode the byte still has to be RECORDED -- it is what
        // full mode will use when it is next selected -- but it must not bump
        // the epoch. The tools drive it from a two-value ladder that re-sends
        // on every keypress, and a bump each time is a camera re-init, with
        // the sensor's settling delay, to write the format it already had.
        wiicam_cam_command("cam=fmt:0");
        const int epb = wiicam_aim_fmt_epoch();
        wiicam_cam_command("cam=fullreg:85");
        ck(wiicam_aim_fmt_epoch() == epb,
           "and changing it in basic mode costs the camera poll nothing");
        ck(wiicam_aim_fullreg() == 0x55 && (wiicam_aim_fmt_state() & 4) == 0
           && wiicam_aim_fmt() == WIICAM_FMT_BASIC,
           "...while still landing in the word, so the next full-mode select "
           "uses the byte the user chose rather than the one before it");
        // Extended is the format the gun actually runs in, so that is where a
        // user setting up full mode would be standing when they flip this.
        wiicam_cam_command("cam=fmt:1");
        const int epe = wiicam_aim_fmt_epoch();
        wiicam_cam_command("cam=fullreg:5");
        ck(wiicam_aim_fmt_epoch() == epe && wiicam_aim_fullreg() == 0x05
           && wiicam_aim_fmt() == WIICAM_FMT_EXT,
           "same in extended: the byte follows the gun into full mode without "
           "having cost it a re-init on the way there");
        wiicam_cam_command("cam=fullreg:85");
        wiicam_cam_command("cam=fmt:0");

        wiicam_set_fullread_hook(0);
    }

    // ---- the SHAPE gate ---------------------------------------------------
    // The 4-bit size is nearly useless on this rig: 52,624 confirmed LED blobs
    // came in at size 1 or 2 with no response to a 1.8x change of distance.
    // Full mode's bounding box and pixel count are a real measurement, and
    // these three knobs gate on them -- as a ONE-CLASS envelope, every bound
    // taken from what an LED has actually looked like, so the gate can only
    // ever refuse something outside everything anyone has measured.
    //
    // Driven through real 37-byte reports, because the numbers it judges reach
    // the gate nowhere else: the box and the pixel count are filled by the
    // full-mode unpack, indexed by HARDWARE SLOT, and the gate has to get at
    // them through the same compaction the rest of the report goes through.
    // Hand it the wrong index and it judges one blob by another blob's shape.
    {
        wiicam_aim_begin();
        wiicam_set_fullread_hook(full_hook);
        g_ffail_on = 0; g_fdrift = 0; g_fhdrdrift = 0;
        wiicam_cam_command("cam=res:0,dash:2,dashhz:0,mirx:1");
        wiicam_cam_command("cam=bmin:0,bmax:15,rtol:0,bhmax:0,pxmax:0,armax:0");
        wiicam_cam_command("cam=fmt:2");

        int gpx[4] = {0, 0, 0, 0}, gpy[4] = {0, 0, 0, 0};
        int gsz[4] = {-1, -1, -1, -1};
        unsigned gseen = 0;
        int gjit = 0;
        // One full-mode frame end to end -- fill the sensor's slots, poll them
        // the way the firmware does, hand the poll's OWN answer to the
        // pipeline -- and report how many blobs came out the far side of the
        // gates. The one pixel of jitter is not cosmetic: a byte-identical
        // report is the previous camera frame seen again and returns the
        // cached answer without touching a single stateful stage, the gate and
        // its counters included, so an unjittered pair would measure one frame
        // and report it as two.
        auto shape_n = [&](const FullObj* o) {
            memcpy(g_fobj, o, sizeof(g_fobj));
            // Every slot, not just the first: the duplicate cache compares
            // only the slots the mask says were SEEN, so jitter parked on an
            // empty slot moves nothing it looks at.
            const int j = (gjit++ & 1) ? 1 : -1;
            for (int k = 0; k < 4; ++k) g_fobj[k].x += j;
            wiicam_aim_full_poll(gpx, gpy, gsz, &gseen);
            g_lines.clear();
            t += DT;
            wiicam_aim_process_sz(gpx, gpy, gsz, gseen, t, &sx, &sy);
            int nn = -1; unsigned long mm;
            if (!g_lines.empty())
                sscanf(g_lines[0].c_str(), "Q,%lu,%d", &mm, &nn);
            return nn;
        };
        // One counter out of '~camblob?'. They are cumulative since boot by
        // design -- the tools show deltas -- so the test reads them that way.
        auto blobstat = [&](const char* key) {
            g_replies.clear();
            wiicam_cam_command("camblob?");
            unsigned long v = 0;
            if (!g_replies.empty()) {
                const char* p = strstr(g_replies[0].c_str(), key);
                if (p) sscanf(p + strlen(key), "%lu", &v);
            }
            return v;
        };

        // Four LEDs on a rectangle, every one of them what this rig actually
        // produces: a 4x4 box at 12 pixels, the very top of the measured
        // envelope. The last column is the report's intensity byte, which is
        // the blob's pixel count and is what pxmax is measured in.
        static const FullObj ROUND[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240, 1,  10, 20, 14, 24, 12 },
            { 768, 240, 1,  10, 20, 14, 24, 12 },
            { 256, 528, 1,  10, 20, 14, 24, 12 },
            { 768, 528, 1,  10, 20, 14, 24, 12 },
        };

        // ---- bhmax: box HEIGHT, and the axis is the whole point ------------
        // Measured over 11,996 confirmed blobs in daylight at sensitivity 2:
        // 99.73% of LEDs come in at a box height of 7 or less and every stray
        // in that capture ran 15 to 56. A cut at 10 caught 84% of them and
        // cost ZERO LEDs -- a gap, not a trade-off. It is the first thing the
        // shape gate looks at, before the pixel count and the ratio.
        wiicam_cam_command("cam=bhmax:8,pxmax:0,armax:0");
        static const FullObj AT_H8[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240, 1,  10, 20, 14, 28, 12 },   // 4 wide, 8 tall
            { 768, 240, 1,  10, 20, 14, 28, 12 },
            { 256, 528, 1,  10, 20, 14, 28, 12 },
            { 768, 528, 1,  10, 20, 14, 28, 12 },
        };
        ck(shape_n(AT_H8) == 4,
           "a blob at EXACTLY bhmax is kept: 8 is the first height a user is "
           "allowed to set and it has to admit a blob of that height, or the "
           "tightest legal cut is one step tighter than it says it is");
        static const FullObj OVER_H8[4] = {
            { 256, 240, 1,  10, 20, 14, 28, 12 },
            { 768, 240, 1,  10, 20, 14, 28, 12 },
            { 256, 528, 1,  10, 20, 14, 28, 12 },
            { 768, 528, 1,  10, 20, 14, 29, 12 },   // 9 tall -- one row over
        };
        {
            const unsigned long s0 = blobstat("bsrej=");
            ck(shape_n(OVER_H8) == 3,
               "and one row over it the blob is dropped before the resolver -- "
               "the whole knob is that boundary");
            ck(blobstat("bsrej=") == s0 + 1,
               "...counted once, in the shape gate's own counter");
        }

        // HEIGHT, not width, and not either-side-will-do. At sensitivity 2 the
        // sensor smears HORIZONTALLY: the same LEDs went from a 2x2 box to
        // 12x3 when the gain went up -- width x5.5, height x1.5. So width
        // measures the gain and height measures the source, which is why this
        // knob replaced the aspect gate rather than joining it. Judge the
        // wrong axis and the gate throws away the smeared LEDs it was set to
        // protect while the tall strays it exists for sail through.
        //
        // The blob COUNT cannot show this: one blob goes either way. Which one
        // can, so that is what is read.
        static const FullObj WIDE_VS_TALL[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240, 1,  10, 20, 50, 23, 12 },   // 40 wide, 3 tall: an LED
            { 768, 240, 1,  10, 20, 14, 32, 12 },   // 4 wide, 12 tall: a stray
            { 256, 528, 1,  10, 20, 14, 24, 12 },   // an ordinary 4x4
            { 768, 528, 1,  10, 20, 14, 24, 12 },
        };
        ck(shape_n(WIDE_VS_TALL) == 3, "a wide-and-short blob beside a tall "
           "narrow one, and exactly one of them goes");
        {
            g_replies.clear();
            wiicam_cam_command("camblob?");
            int hb[27];
            int goth = 0;
            if (g_replies.size() >= 2)
                goth = sscanf(g_replies[1].c_str(),
                              "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                              &hb[0],&hb[1],&hb[2],&hb[3],&hb[4],&hb[5],&hb[6],&hb[7],&hb[8],
                              &hb[9],&hb[10],&hb[11],&hb[12],&hb[13],&hb[14],&hb[15],&hb[16],&hb[17],
                              &hb[18],&hb[19],&hb[20],&hb[21],&hb[22],&hb[23],&hb[24],&hb[25],&hb[26]);
            ck(goth == 27 && hb[4] == 40 && hb[5] == 3 && hb[3] == 1,
               "the 40-wide, 3-tall blob SURVIVES a height cut of 8 -- ten "
               "times over the limit on the axis the gate does not judge, and "
               "that is a smeared LED, which is the blob this knob exists to "
               "keep");
            ck(goth == 27 && hb[13] == 4 && hb[14] == 12 && hb[12] == 0,
               "...and the 4-wide, 12-tall one is the one that goes: height is "
               "what is measured, so the narrow blob fails the cut the wide "
               "one passed. Judge width instead and this test still drops "
               "exactly one blob -- the wrong one");
        }

        // ---- the height gate reaches the box through the SAME compaction ---
        // Same bug as the pixel count's, and this is the one that shipped: the
        // box arrays are filled by HARDWARE SLOT and everything the report
        // shows walks the COMPACTED seen list, so a LEADING empty slot puts
        // the two out of step. Slot 0 goes empty exactly when a bright source
        // has taken it, which is the case the gate exists for.
        static const FullObj GAP_TALL[4] = {
        //    x     y  sz  xmn ymn xmx ymx  px
            { 1023,1023, 15,  0,  0,  0,  0,  0 },   // EMPTY
            {  256, 240,  1, 10, 20, 14, 32, 12 },   // the offender: 12 tall
            {  768, 240,  1, 10, 20, 14, 24, 12 },
            {  256, 528,  1, 10, 20, 14, 24, 12 },
        };
        ck(shape_n(GAP_TALL) == 2,
           "with slot 0 empty and one of the three remaining blobs too tall, "
           "two survive");
        {
            g_replies.clear();
            wiicam_cam_command("camblob?");
            int hb[27];
            int goth = 0;
            if (g_replies.size() >= 2)
                goth = sscanf(g_replies[1].c_str(),
                              "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                              &hb[0],&hb[1],&hb[2],&hb[3],&hb[4],&hb[5],&hb[6],&hb[7],&hb[8],
                              &hb[9],&hb[10],&hb[11],&hb[12],&hb[13],&hb[14],&hb[15],&hb[16],&hb[17],
                              &hb[18],&hb[19],&hb[20],&hb[21],&hb[22],&hb[23],&hb[24],&hb[25],&hb[26]);
            ck(goth == 27 && hb[5] == 12 && hb[14] == 4 && hb[23] == 4,
               "the FIRST listed blob is slot 1, and it is the one carrying "
               "the 12-row box -- without that this check is measuring the "
               "report's compaction rather than the gate's");
            ck(goth == 27 && hb[3] == 0 && hb[12] == 1 && hb[21] == 1,
               "...and it is the first listed blob the gate rejected. Read by "
               "the compacted index instead, slot 0's cleared height of 0 "
               "acquits it and the innocent blob one place down the list takes "
               "the verdict -- two blobs wrong, the same count, and the "
               "earlier version of exactly this bug shipped once already");
        }

        // Outside full mode there is no box, so there is nothing to judge and
        // the gate stands down -- knob still set, box still sitting in the
        // arrays from the last 37-byte read, because the poll on the other
        // core goes on doing them until it notices the new epoch.
        wiicam_cam_command("cam=fmt:1");
        ck(shape_n(WIDE_VS_TALL) == 4,
           "in extended the height gate does nothing at all, bhmax still set "
           "and a freshly unpacked box sitting in the arrays");
        wiicam_cam_command("cam=fmt:0");
        ck(shape_n(WIDE_VS_TALL) == 4, "nor in basic, for the same reason");
        wiicam_cam_command("cam=fmt:2");
        ck(shape_n(WIDE_VS_TALL) == 3,
           "...and full mode turns it straight back on, so what stood it down "
           "was the format and not some latch it never got out of");

        // The size window gets first refusal here too. A blob it has already
        // thrown out is not the height gate's to throw out again: double
        // counted, bsrej reads as evidence that bhmax is earning its place
        // when the window did all the work, and bsrej is the number the knob
        // gets tuned from.
        wiicam_cam_command("cam=bmin:0,bmax:8");
        static const FullObj TALL_AND_BIG[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240,  1, 10, 20, 14, 24, 12 },
            { 768, 240,  1, 10, 20, 14, 24, 12 },
            { 256, 528,  1, 10, 20, 14, 24, 12 },
            { 768, 528, 14, 10, 20, 14, 32, 12 },   // size 14 AND 12 tall
        };
        {
            const unsigned long w0 = blobstat("brej=");
            const unsigned long s0 = blobstat("bsrej=");
            ck(shape_n(TALL_AND_BIG) == 3,
               "a blob that fails the size window and the height cut is "
               "dropped -- once");
            ck(blobstat("brej=") == w0 + 1,
               "...by the window, which saw it first");
            ck(blobstat("bsrej=") == s0,
               "...and NOT counted again by the height gate, which never looks "
               "at a blob that is already gone");
        }
        wiicam_cam_command("cam=bmin:0,bmax:15,bhmax:0");

        // ---- pxmax, and which side of it the bound falls on ----------------
        wiicam_cam_command("cam=pxmax:12,armax:0");
        ck(shape_n(ROUND) == 4,
           "a blob at EXACTLY pxmax is kept: the envelope was measured up to "
           "12 pixels inclusive, so a '>=' here refuses the largest LED "
           "anyone has ever recorded off this rig");
        static const FullObj ONE_OVER[4] = {
            { 256, 240, 1,  10, 20, 14, 24, 12 },
            { 768, 240, 1,  10, 20, 14, 24, 12 },
            { 256, 528, 1,  10, 20, 14, 24, 12 },
            { 768, 528, 1,  10, 20, 14, 24, 13 },   // one pixel too many
        };
        {
            const unsigned long s0 = blobstat("bsrej=");
            ck(shape_n(ONE_OVER) == 3,
               "and one pixel over it the blob is dropped before the resolver "
               "-- the whole knob is that boundary");
            ck(blobstat("bsrej=") == s0 + 1,
               "...counted once, in the shape gate's own counter");
        }

        // ---- armax, in BOTH orientations -----------------------------------
        // The ratio is longest side over shortest, not width over height. A
        // w/h would let every TALL stray through and an h/w every wide one,
        // and either mistake leaves a gate that looks like it works because
        // half the strays a bench test throws at it still get caught.
        wiicam_cam_command("cam=pxmax:0,armax:16");
        static const FullObj AT_2TO1[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240, 1,  10, 20, 14, 28, 12 },   // 4 wide, 8 tall -- 2:1
            { 768, 240, 1,  10, 20, 18, 24, 12 },   // 8 wide, 4 tall -- 2:1
            { 256, 528, 1,  10, 20, 14, 24, 12 },   // 4x4
            { 768, 528, 1,  10, 20, 14, 24, 12 },   // 4x4
        };
        ck(shape_n(AT_2TO1) == 4,
           "exactly at armax is kept, tall as well as wide -- 99.9% of the "
           "measured LEDs are 2:1 or rounder, so 16 eighths is the last ratio "
           "the envelope has to admit rather than the first it refuses");
        static const FullObj TALL[4] = {
            { 256, 240, 1,  10, 20, 14, 24, 12 },
            { 768, 240, 1,  10, 20, 14, 24, 12 },
            { 256, 528, 1,  10, 20, 14, 24, 12 },
            { 768, 528, 1,  10, 20, 14, 29, 12 },   // 4 wide, 9 tall -- 2.25:1
        };
        ck(shape_n(TALL) == 3,
           "a TALL blob past armax is dropped -- judged as width over height "
           "its 4 by 9 reads as rounder than round and it sails through");
        static const FullObj WIDE[4] = {
            { 256, 240, 1,  10, 20, 14, 24, 12 },
            { 768, 240, 1,  10, 20, 14, 24, 12 },
            { 256, 528, 1,  10, 20, 14, 24, 12 },
            { 768, 528, 1,  10, 20, 19, 24, 12 },   // 9 wide, 4 tall -- 2.25:1
        };
        ck(shape_n(WIDE) == 3,
           "and so is a WIDE one -- height over width would have missed this "
           "one instead, which is why both orientations are checked and not "
           "whichever one the first stray happened to be");

        // ---- a zero box side is not an infinitely elongated blob ------------
        // It is the SMALLEST thing the sensor can report: a blob one pixel
        // across in that axis, which is a faint LED at the far end of the play
        // range. There is no ratio to take, and taking one anyway divides by
        // zero on the way to rejecting the whole distant half of the room.
        static const FullObj FLAT[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240, 1,  10, 20, 10, 26, 12 },   // zero WIDTH, 6 tall
            { 768, 240, 1,  10, 20, 22, 20, 12 },   // 12 wide, zero HEIGHT
            { 256, 528, 1,  10, 20, 10, 20, 12 },   // zero both ways
            { 768, 528, 1,  10, 20, 14, 24, 12 },   // an ordinary 4x4
        };
        ck(shape_n(FLAT) == 4,
           "a zero-width, a zero-height and a zero-by-zero box are all kept "
           "with armax at its tightest: a side the sensor rounded to nothing "
           "is the faintest LED it can see, not a streak");

        // ---- the gate reaches the box through the SAME compaction -----------
        // The poll fills its box arrays by HARDWARE SLOT, 0..3 as the sensor
        // numbers them; everything the report shows walks the COMPACTED seen
        // list, because process_sz skips empty slots as it goes. Index the box
        // by the compacted position and the gate judges the first blob by
        // slot 0's box -- which belongs to no blob on screen the moment any
        // slot but the last is empty. A slot goes empty exactly when a bright
        // source has taken it, so the gate would be reading the wrong shape in
        // precisely the case it exists for.
        //
        // The blob COUNT cannot show this: one blob is rejected either way.
        // Which blob got the verdict can, so that is what is read.
        wiicam_cam_command("cam=pxmax:12,armax:0");
        static const FullObj GAP_FAT[4] = {
        //    x     y  sz  xmn ymn xmx ymx  px
            { 1023,1023, 15,  0,  0,  0,  0,  0 },   // EMPTY
            {  256, 240,  1, 10, 20, 14, 24, 40 },   // the offender: 40 pixels
            {  768, 240,  1, 10, 20, 14, 24, 12 },
            {  256, 528,  1, 10, 20, 14, 24, 12 },
        };
        ck(shape_n(GAP_FAT) == 2,
           "with slot 0 empty and one of the three remaining blobs too fat, "
           "two survive");
        {
            g_replies.clear();
            wiicam_cam_command("camblob?");
            int gb[27];
            int gotg = 0;
            if (g_replies.size() >= 2)
                gotg = sscanf(g_replies[1].c_str(),
                              "CAM: blobs %d,%d,%d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d,%d,%d",
                              &gb[0],&gb[1],&gb[2],&gb[3],&gb[4],&gb[5],&gb[6],&gb[7],&gb[8],
                              &gb[9],&gb[10],&gb[11],&gb[12],&gb[13],&gb[14],&gb[15],&gb[16],&gb[17],
                              &gb[18],&gb[19],&gb[20],&gb[21],&gb[22],&gb[23],&gb[24],&gb[25],&gb[26]);
            ck(gotg == 27 && gb[6] == 40 && gb[15] == 12 && gb[24] == 12,
               "the FIRST listed blob is slot 1, and it is the one carrying "
               "the 40 pixels -- without that this check is measuring the "
               "report's compaction rather than the gate's");
            ck(gotg == 27 && gb[3] == 0 && gb[12] == 1 && gb[21] == 1,
               "...and it is the first listed blob the gate rejected: judged "
               "by the compacted index instead, slot 0's cleared box would "
               "have acquitted it and an innocent blob further down the list "
               "would have taken the verdict in its place");
        }

        // ---- two knobs at once, and then the format that switches them off --
        wiicam_cam_command("cam=bhmax:0,pxmax:12,armax:16");
        static const FullObj MIXED[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240, 1,  10, 20, 14, 24, 12 },   // an LED
            { 768, 240, 1,  10, 20, 14, 24, 12 },   // an LED
            { 256, 528, 1,  10, 20, 14, 24, 40 },   // 4x4 but 40 pixels
            { 768, 528, 1,  10, 20, 30, 24, 12 },   // 20x4 -- 5:1
        };
        ck(shape_n(MIXED) == 2,
           "the two knobs act on the same keep[] in the same pass: the fat "
           "blob and the long one both go, and neither knob undoes the "
           "other's verdict");

        // Outside full mode the box arrays mean nothing and the gate has to
        // stand down ENTIRELY. The case that matters is not the tidy one where
        // those arrays are empty: the command parser runs on one core and the
        // camera poll on the other, so the poll goes on doing 37-byte reads
        // until it notices the new epoch, and a frame reaches process_sz with
        // a real full-mode box in the arrays while the format word already
        // says extended. Judge a blob by a box the format says the sensor is
        // not sending and the gate is acting on a number whose meaning it
        // cannot know -- which is how a gate rejects everything.
        wiicam_cam_command("cam=fmt:1");
        ck(shape_n(MIXED) == 4,
           "in extended the shape gate does nothing at all, both knobs still "
           "set and a freshly unpacked box sitting in the arrays");
        wiicam_cam_command("cam=fmt:0");
        ck(shape_n(MIXED) == 4, "nor in basic, for the same reason");
        wiicam_cam_command("cam=fmt:2");
        ck(shape_n(MIXED) == 2,
           "...and full mode turns it straight back on, so what stood the "
           "gate down was the format and not some latch it never got out of");

        // ---- all THREE knobs live at once -----------------------------------
        // Three tests in one pass over the same keep[], and the failure worth
        // catching is not that one of them stops working -- it is that they
        // interfere. A blob one knob condemned and another quietly acquitted
        // looks exactly like a knob that is simply set too loose, and a blob
        // counted once per knob it fails inflates bsrej by most on the frames
        // where the strays are worst.
        //
        // One offender per frame, of a different kind each time, with all
        // three knobs set the whole way through. Each blob is built to fail
        // ONE test and pass the other two: the 12x12 is perfectly square and
        // 12 pixels, so only its height is out; the 4x4 at 40 pixels is short
        // and round; the 20x4 is short and 12 pixels.
        wiicam_cam_command("cam=bhmax:8,pxmax:12,armax:16");
        static const FullObj TALL_ONE[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240, 1,  10, 20, 14, 24, 12 },
            { 768, 240, 1,  10, 20, 14, 24, 12 },
            { 256, 528, 1,  10, 20, 14, 24, 12 },
            { 768, 528, 1,  10, 20, 22, 32, 12 },   // 12x12: too TALL only
        };
        static const FullObj FAT_ONE[4] = {
            { 256, 240, 1,  10, 20, 14, 24, 12 },
            { 768, 240, 1,  10, 20, 14, 24, 12 },
            { 256, 528, 1,  10, 20, 14, 24, 12 },
            { 768, 528, 1,  10, 20, 14, 24, 40 },   // 4x4 at 40px: too FAT only
        };
        static const FullObj LONG_ONE[4] = {
            { 256, 240, 1,  10, 20, 14, 24, 12 },
            { 768, 240, 1,  10, 20, 14, 24, 12 },
            { 256, 528, 1,  10, 20, 14, 24, 12 },
            { 768, 528, 1,  10, 20, 30, 24, 12 },   // 20x4, 5:1: too LONG only
        };
        {
            const unsigned long s0 = blobstat("bsrej=");
            ck(shape_n(TALL_ONE) == 3,
               "with all three knobs set, the height cut drops its own blob "
               "and neither of the other two rescues it");
            ck(shape_n(FAT_ONE) == 3,
               "...the pixel count drops its own, past a height cut that is "
               "perfectly happy with a 4x4");
            ck(shape_n(LONG_ONE) == 3,
               "...and the ratio drops its own, past a height cut that is "
               "perfectly happy with a 20x4 -- which is also why the ratio is "
               "still worth having beside a knob that only looks at height");
            ck(blobstat("bsrej=") == s0 + 3,
               "three frames, three rejections, all in the one counter the "
               "shape gate is tuned from");
        }
        static const FullObj TALL_AND_FAT[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240, 1,  10, 20, 14, 24, 12 },
            { 768, 240, 1,  10, 20, 14, 24, 12 },
            { 256, 528, 1,  10, 20, 14, 24, 12 },
            { 768, 528, 1,  10, 20, 22, 32, 40 },   // 12x12 AND 40 pixels
        };
        {
            const unsigned long s0 = blobstat("bsrej=");
            ck(shape_n(TALL_AND_FAT) == 3,
               "and a blob that fails TWO of the three is still one blob");
            ck(blobstat("bsrej=") == s0 + 1,
               "...counted ONCE. Counted per knob it fails, bsrej would run "
               "ahead of the number of blobs there were, and the readout the "
               "gate is set from would say the shape knobs were catching more "
               "than the sensor ever sent");
        }

        // Off is off. All three knobs at 0 is the shipped state, and it has to
        // be the state a user can get back to by hand.
        wiicam_cam_command("cam=bhmax:0,pxmax:0,armax:0");
        {
            const unsigned long s0 = blobstat("bsrej=");
            ck(shape_n(TALL_AND_FAT) == 4,
               "with all three knobs at 0 the gate is inert, in full mode, on "
               "the frame it had just been shredding");
            ck(blobstat("bsrej=") == s0,
               "...and counts nothing either: a gate that is off must not be "
               "reporting rejections it did not make");
        }

        // ---- the size window gets first refusal, and keeps it ---------------
        // The two gates run on one keep[] array, the window first. A blob it
        // has already thrown out is not the shape gate's to throw out again --
        // double-counted, bsrej reads as evidence that the shape knobs are
        // earning their place when in fact the window did all the work, and
        // the readout the gate has to be TUNED from says the opposite of the
        // truth.
        wiicam_cam_command("cam=bmin:0,bmax:8,pxmax:12,armax:16");
        static const FullObj ALREADY_OUT[4] = {
        //    x    y  sz  xmn ymn xmx ymx  px
            { 256, 240,  1, 10, 20, 14, 24, 12 },
            { 768, 240,  1, 10, 20, 14, 24, 12 },
            { 256, 528,  1, 10, 20, 14, 24, 12 },
            { 768, 528, 14, 10, 20, 30, 24, 40 },   // out of the window AND
        };                                          // 5:1 at 40 pixels
        {
            const unsigned long w0 = blobstat("brej=");
            const unsigned long s0 = blobstat("bsrej=");
            ck(shape_n(ALREADY_OUT) == 3,
               "a blob that fails the window and both shape knobs is dropped "
               "-- once");
            ck(blobstat("brej=") == w0 + 1,
               "...by the window, which saw it first");
            ck(blobstat("bsrej=") == s0,
               "...and NOT counted again by the shape gate: bsrej is what the "
               "shape knobs are worth on their own, and it is the number they "
               "get tuned from");
        }
        wiicam_cam_command("cam=bmin:0,bmax:15");

        // ---- the floor is still downstream of it ----------------------------
        // Below TWO points the resolver cannot fit anything and GetPosition
        // falls through to the stock uncalibrated path -- the cursor jumps. A
        // shape gate set so tight it would do that is set wrong, and giving
        // blobs back is the correct response; running the gate AFTER the floor
        // instead would let it undo the one guarantee the floor exists to make.
        wiicam_cam_command("cam=pxmax:12,armax:0");
        static const FullObj ALL_FAT[4] = {
            { 256, 240, 1,  10, 20, 14, 24, 40 },
            { 768, 240, 1,  10, 20, 14, 24, 40 },
            { 256, 528, 1,  10, 20, 14, 24, 40 },
            { 768, 528, 1,  10, 20, 14, 24, 40 },
        };
        {
            const unsigned long s0 = blobstat("bsrej=");
            const unsigned long v0 = blobstat("bvalve=");
            ck(shape_n(ALL_FAT) == 2,
               "a shape gate that rejects EVERY blob is held to the same floor "
               "of two, never zero -- it cannot blind the gun any more than "
               "the size window can");
            ck(blobstat("bsrej=") == s0 + 4,
               "...with all FOUR rejections counted, before the floor gave any "
               "back: a count taken afterwards would report two on a gate that "
               "refused everything, which is idle-looking in exactly the case "
               "the number exists to reveal");
            ck(blobstat("bvalve=") == v0 + 2,
               "...and the two that came back are the FLOOR's doing, recorded "
               "where a user tuning the knob can see the gate is being "
               "overruled");
        }
        wiicam_cam_command("cam=pxmax:0,armax:0");

        // ---- where the counter is reported ----------------------------------
        {
            g_replies.clear();
            wiicam_cam_command("camblob?");
            const std::string l1 = g_replies.empty() ? std::string() : g_replies[0];
            const size_t at_drop = l1.find("bdrop=");
            const size_t at_srej = l1.find("bsrej=");
            const size_t at_far  = l1.find("bfar=");
            ck(at_drop != std::string::npos && at_srej != std::string::npos
               && at_far != std::string::npos
               && at_drop < at_srej && at_srej < at_far,
               "bsrej sits between bdrop and bfar on the blob line -- the "
               "tools read this line positionally as well as by name, and a "
               "counter inserted anywhere else shifts every column after it");
        }

        // ---- the refusals ----------------------------------------------------
        // Refused BELOW the measured envelope rather than clamped up to it. A
        // bhmax under 8, a pxmax under 12 or an armax under 16 would reject
        // blobs we have watched real LEDs produce, and a gate that can do that
        // is not a gate, it is an outage waiting for the right angle. Clamping
        // would hide the mistake; refusing by name puts it on the wire.
        // Standing at 10, 14 and 20 rather than at the bounds themselves, so
        // "the value did not move" can tell a refusal apart from a silent
        // clamp up to the bound -- which is the shape this mistake actually
        // takes, and which would look identical from a gate standing at 8, 12
        // and 16.
        wiicam_cam_command("cam=bhmax:10,pxmax:14,armax:20");
        g_replies.clear();
        wiicam_cam_command("cam=pxmax:11");
        ck(!g_replies.empty()
           && g_replies[0].find("pxmax below 12") != std::string::npos,
           "pxmax:11 is refused BY NAME -- one under the envelope is exactly "
           "the value a spinner lands on and it would start dropping real "
           "LEDs at the near end of the play range");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("pxmax=14 ") != std::string::npos,
           "...and the value does not move -- not to 11, and not quietly up to "
           "12 either: a refusal that half-applied would leave behind a gate "
           "nobody asked for");
        g_replies.clear();
        wiicam_cam_command("cam=armax:15");
        ck(!g_replies.empty()
           && g_replies[0].find("armax below 16") != std::string::npos,
           "armax:15 likewise -- 15 eighths is under 2:1, and 2:1 is a shape "
           "the measured LEDs actually reach");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("armax=20 ") != std::string::npos,
           "...and it stays where it was, at 20 and not at the bound");

        // bhmax's floor is 8, and it is not arbitrary: across 11,996 confirmed
        // blobs in daylight 99.73% of LEDs came in at a box height of 7 or
        // less, and the strays ran 15 to 56. Seven is one step into the LEDs
        // and it is the value a spinner walking down from 8 lands on.
        g_replies.clear();
        wiicam_cam_command("cam=bhmax:7");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax below 8") != std::string::npos,
           "bhmax:7 is refused BY NAME -- one row under the envelope starts "
           "eating the measured LEDs, and the whole reason this knob is worth "
           "having is that at 8 and above it costs none of them");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=10 ") != std::string::npos,
           "...and the value does not move -- not to 7, and not quietly up to "
           "8 either: standing at 10 is what tells a refusal apart from a "
           "clamp, and a clamp would leave behind the tightest legal gate on "
           "a gun whose user was trying to loosen it");

        // The envelope itself is the first legal value on each knob.
        g_replies.clear();
        wiicam_cam_command("cam=bhmax:8,pxmax:12,armax:16");
        ck(g_replies.size() == 1
           && g_replies[0].find("CMD ok (tune)") != std::string::npos,
           "8, 12 and 16 -- the envelope itself -- are accepted with no "
           "refusal line at all");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=8 ") != std::string::npos
           && g_replies[0].find("pxmax=12 ") != std::string::npos
           && g_replies[0].find("armax=16 ") != std::string::npos,
           "...and they really land, which is also what says the refusals "
           "above left 10, 14 and 20 in place rather than never writing at "
           "all");
        // The two values a tool's own ladder offers above the bound. 10 is the
        // cut the capture actually recommends -- 84% of the strays, zero LEDs
        // -- so it had better be typeable.
        g_replies.clear();
        wiicam_cam_command("cam=bhmax:10");
        ck(g_replies.size() == 1
           && g_replies[0].find("not set") == std::string::npos
           && g_replies[0].find("bhmax=10 ") != std::string::npos,
           "bhmax:10, the cut the daylight capture recommends, is accepted");
        g_replies.clear();
        wiicam_cam_command("cam=bhmax:12");
        ck(g_replies.size() == 1
           && g_replies[0].find("not set") == std::string::npos
           && g_replies[0].find("bhmax=12 ") != std::string::npos,
           "and so is bhmax:12 -- the refusal is a floor, not a whitelist of "
           "the two values someone happened to try");
        wiicam_cam_command("cam=bhmax:10,pxmax:14,armax:20");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=10 ") != std::string::npos
           && g_replies[0].find("pxmax=14 ") != std::string::npos
           && g_replies[0].find("armax=20 ") != std::string::npos,
           "and so are 10, 14 and 20, a step of margin outside it -- the gate "
           "is only ever allowed to be LOOSER than what was measured");

        // Zero is always legal, on every knob, from any value. It is off, and
        // off is the state a user reaches for when the gate is the suspect.
        g_replies.clear();
        wiicam_cam_command("cam=pxmax:0");
        ck(g_replies.size() == 1
           && g_replies[0].find("not set") == std::string::npos,
           "pxmax:0 is not refused: 0 is off, not a gate below the envelope");
        g_replies.clear();
        wiicam_cam_command("cam=armax:0");
        ck(g_replies.size() == 1
           && g_replies[0].find("not set") == std::string::npos,
           "and neither is armax:0");
        g_replies.clear();
        wiicam_cam_command("cam=bhmax:0");
        ck(g_replies.size() == 1
           && g_replies[0].find("not set") == std::string::npos,
           "nor bhmax:0 -- 0 is below 8 arithmetically and it is the one value "
           "below 8 that must never be refused, because it is how the knob is "
           "switched off");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=0 ") != std::string::npos
           && g_replies[0].find("pxmax=0 ") != std::string::npos
           && g_replies[0].find("armax=0 ") != std::string::npos,
           "...and all three really do land, so the one way out of a bad shape "
           "gate is reachable by hand");

        // Out of range at the top clamps rather than wrapping: all three
        // fields are six bits in the stored word, and an unclamped 200 comes
        // back as 8.
        wiicam_cam_command("cam=bhmax:99,pxmax:999,armax:1000");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=63 ") != std::string::npos
           && g_replies[0].find("pxmax=63 ") != std::string::npos
           && g_replies[0].find("armax=63 ") != std::string::npos,
           "a value past the top clamps to 63, the widest the stored field "
           "holds -- wrapping it would turn an absurdly loose gate into a "
           "tight one nobody asked for");
        wiicam_cam_command("cam=bhmax:-5,pxmax:-5,armax:-5");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=0 ") != std::string::npos
           && g_replies[0].find("pxmax=0 ") != std::string::npos
           && g_replies[0].find("armax=0 ") != std::string::npos,
           "and a negative one is off, not a refusal: the clamp runs first, so "
           "typing a minus to mean 'no limit' does what it looks like, on "
           "bhmax as well -- a -5 that reached the floor check would be "
           "refused for being under 8 and the minus would do nothing");

        // ---- where the three knobs are reported ------------------------------
        // Position, not just presence. The tools read these lines by name AND
        // positionally, so a key inserted anywhere but where it is documented
        // shifts every column after it in every log drawn from them. bhmax
        // goes immediately before pxmax on all three lines.
        wiicam_cam_command("cam=bhmax:10,pxmax:14,armax:20");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=10 pxmax=14 armax=20 ")
              != std::string::npos,
           "cam? carries all three, bhmax first and immediately before pxmax");
        g_replies.clear();
        wiicam_cam_command("camblob?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=10 pxmax=14 armax=20 ")
              != std::string::npos,
           "so does the blob line, in the same order, so the capture a gate is "
           "chosen from records the gate that was in force while it was taken");
        g_replies.clear();
        wiicam_cam_command("cam=armax:20");
        ck(!g_replies.empty()
           && g_replies.back().find("CMD ok (tune)") != std::string::npos
           && g_replies.back().find("bhmax=10 pxmax=14 armax=20 ")
              != std::string::npos,
           "and so does the tune echo, which is the only reply a tool gets "
           "back from a 'cam=' line and therefore the only place it can read "
           "what actually took");

        wiicam_cam_command("cam=bhmax:0,pxmax:0,armax:0,fmt:0");
        wiicam_set_fullread_hook(0);
    }

    // ---- the gate that survives a power cycle -----------------------------
    // The first real capture at the TV had the gun holding 99.7% four-corner
    // with rtol on, and every one of those settings died on the next power
    // cycle -- including the FORMAT, without which the size arrives as -1 and
    // the gate silently judges nothing at all. All four go into ONE tagged
    // word: four separate keys can come back half-written, and a tolerance
    // without the format that feeds it reads as configured while gating
    // nothing.
    {
        int gf = -1, gmn = -1, gmx = -1, gt = -1;
        ck(aim_gate_store(2, 4, 11, 6), "the gate stores as one word");
        ck(aim_gate_load(&gf, &gmn, &gmx, &gt), "and loads back");
        ck(gf == 2 && gmn == 4 && gmx == 11 && gt == 6,
           "with all four fields intact, each in its own nibble");
        // Every field is four bits, so an unclamped value does not merely come
        // back wrong -- it bleeds into the field above it.
        ck(aim_gate_store(9, -3, 99, 40), "an out-of-range gate still stores");
        ck(aim_gate_load(&gf, &gmn, &gmx, &gt)
           && gf == 2 && gmn == 0 && gmx == 15 && gt == 15,
           "clamped on the way in, so a bmax of 99 cannot land in the format "
           "nibble above it");
        // A foreign or stale key must read as NOTHING STORED. Read as a gate
        // it would be 0,0,0,0 -- the size window shut to a single value, on a
        // gun that had a working one.
        nvs_set_u32(1, "gate0", 0x00001234u);
        gf = gmn = gmx = gt = -1;
        ck(!aim_gate_load(&gf, &gmn, &gmx, &gt),
           "a word without the tag is 'nothing stored', not a gate of zeroes");
        ck(gf == -1 && gmn == -1 && gmx == -1 && gt == -1,
           "and the caller's own defaults are left exactly where they were");
        ck(aim_gate_clear(), "clearing it succeeds");
        ck(!aim_gate_load(&gf, &gmn, &gmx, &gt), "after which nothing is stored");
        ck(aim_gate_clear(),
           "and clearing an ALREADY absent key is not a failure -- camreset "
           "would otherwise report that it could not erase a gate that was "
           "never there");

        // End to end: camsave writes it, a reboot brings it back, camreset
        // takes it away again.
        wiicam_aim_begin();
        wiicam_cam_command("cam=fmt:1,bmin:4,bmax:11,rtol:6");
        g_replies.clear();
        wiicam_cam_command("camsave");
        ck(!g_replies.empty()
           && g_replies[0].find("fmt=1") != std::string::npos
           && g_replies[0].find("bmin=4") != std::string::npos
           && g_replies[0].find("bmax=11") != std::string::npos
           && g_replies[0].find("rtol=6") != std::string::npos,
           "camsave names the gate it wrote, so a tool can VERIFY it rather "
           "than assume it");
        wiicam_cam_command("cam=fmt:0,bmin:0,bmax:15,rtol:0");   // forget it
        const int epg = wiicam_aim_fmt_epoch();
        wiicam_aim_begin();
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("fmt=1") != std::string::npos
           && g_replies[0].find("bmin=4") != std::string::npos
           && g_replies[0].find("bmax=11") != std::string::npos
           && g_replies[0].find("rtol=6") != std::string::npos,
           "a reboot brings the whole gate back, the format included");
        ck(wiicam_aim_fmt_epoch() != epg && wiicam_aim_fmt() == WIICAM_FMT_EXT,
           "and it arrives with a FRESH epoch, so the camera poll really "
           "applies it -- a restored format that never reaches the sensor is "
           "a read length that does not match the report");

        // Full mode does NOT get to persist. It is unproven -- we do not even
        // know for certain which byte selects it -- and a gun that boots into
        // a format the sensor does not honour reads a length that matches no
        // report, decodes plausible nonsense and aims wildly, with no console
        // on a Pi to say why. Same rule as hwmax/hwmin: anything that can
        // leave the gun unusable stays one power cycle from gone. Lift it once
        // full mode is confirmed on hardware.
        wiicam_cam_command("cam=fmt:2");
        g_replies.clear();
        wiicam_cam_command("camsave");
        ck(!g_replies.empty()
           && g_replies[0].find("fmt=1") != std::string::npos,
           "camsave with full mode live stores EXTENDED, and the reply says "
           "fmt=1 so the tool reports what was written, not what was asked");
        ck(wiicam_aim_fmt() == WIICAM_FMT_FULL,
           "...without disturbing the live session, which keeps full mode for "
           "as long as the gun stays powered");
        {
            int sf = -1, smn = -1, smx = -1, srt = -1;
            ck(aim_gate_load(&sf, &smn, &smx, &srt) && sf == WIICAM_FMT_EXT,
               "and the stored word really holds 1: the reply is not the only "
               "place the clamp has to happen, and it is not the one that "
               "decides what the next boot does");
        }
        wiicam_aim_begin();
        ck(wiicam_aim_fmt() == WIICAM_FMT_EXT,
           "so the boot after a camsave in full mode comes up in extended, "
           "which this sensor is known to honour");
        g_replies.clear();
        wiicam_cam_command("camreset");
        ck(!aim_gate_load(&gf, &gmn, &gmx, &gt),
           "camreset erases the SAVED copy too: now that the gate survives a "
           "reboot, a gate that stops the gun aiming would come back on the "
           "next boot and the one command a user reaches for when nothing "
           "works would fix the session and lose the argument");
        ck(!g_replies.empty()
           && g_replies[0].find("unsaved") != std::string::npos,
           "...and the reply says so, instead of claiming an erase that did "
           "not happen");
        wiicam_aim_begin();
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(wiicam_aim_fmt() == WIICAM_FMT_BASIC
           && !g_replies.empty()
           && g_replies[0].find("bmin=0") != std::string::npos
           && g_replies[0].find("bmax=15") != std::string::npos
           && g_replies[0].find("rtol=0") != std::string::npos,
           "so the boot after that one really does come up inert");
    }

    // ---- the shape gate's OWN key -----------------------------------------
    // A SECOND word rather than two more fields in the first. The first is
    // full at 14 bits of payload under its tag, and re-packing it would make
    // every gate already sitting in a gun's flash unreadable -- a silent
    // downgrade to "no gate stored" on the one setting that survives a reboot,
    // on exactly the guns that had bothered to save one.
    //
    // Which buys nothing unless the two keys really are independent, so that
    // is most of what is checked here: neither may answer for the other, erase
    // the other, or be readable out of the other's bits.
    {
        int gp = -1, ga = -1, gb = -1;
        ck(aim_gate2_store(12, 20, 34), "the shape gate stores as one word");
        ck(aim_gate2_load(&gp, &ga, &gb), "and loads back");
        ck(gp == 12 && ga == 20 && gb == 34,
           "with all three knobs intact and none of them swapped -- bhmax, "
           "pxmax and armax are the same width and none of the three values is "
           "out of range for the other two, so any permutation of them is "
           "three plausible numbers");
        // Six bits each, so an unclamped value does not merely come back
        // wrong: it bleeds into the field above it.
        ck(aim_gate2_store(-3, 99, 200), "an out-of-range triple still stores");
        ck(aim_gate2_load(&gp, &ga, &gb) && gp == 0 && ga == 63 && gb == 63,
           "clamped on the way in, each on its own, so an armax of 99 cannot "
           "land a carry bit in the pxmax field above it -- nor a bhmax of 200 "
           "spill out of the top field -- and turn a gate nobody set into one "
           "that rejects blobs");
        // Aliasing, one field at a time. Three fields packed into one word can
        // come back right on a store of three DIFFERENT values and still be
        // wrong: two of them sharing bits reads correctly whenever the shared
        // bits happen to agree. A one-hot store cannot hide that -- the value
        // has to appear in its own slot and in NEITHER of the other two.
        ck(aim_gate2_store(9, 0, 0)
           && aim_gate2_load(&gp, &ga, &gb) && gp == 9 && ga == 0 && gb == 0,
           "a pxmax on its own reads back as a pxmax on its own");
        ck(aim_gate2_store(0, 9, 0)
           && aim_gate2_load(&gp, &ga, &gb) && gp == 0 && ga == 9 && gb == 0,
           "...and an armax on its own as an armax");
        ck(aim_gate2_store(0, 0, 9)
           && aim_gate2_load(&gp, &ga, &gb) && gp == 0 && ga == 0 && gb == 9,
           "...and a bhmax on its own as a bhmax: the field added last is the "
           "one that could have been laid on top of an existing one, and a gun "
           "with only bhmax set would then boot with a pixel-count gate it was "
           "never given");
        // A stale or foreign word must read as NOTHING STORED. The tag is the
        // only thing that can tell the difference, and it is the whole reason
        // the second key could be added without disturbing the first.
        nvs_set_u32(1, "gate1", 0x00005678u);
        gp = ga = gb = -1;
        ck(!aim_gate2_load(&gp, &ga, &gb),
           "an untagged word is 'nothing stored', not a shape gate");
        ck(gp == -1 && ga == -1 && gb == -1,
           "and the caller's own defaults are left exactly where they were -- "
           "wiicam_aim_begin passes in the shipped 0,0,0 and a load that wrote "
           "before it failed would gate a gun that never asked for it");
        // A gun flashed while the shape gate had only TWO fields. Its 'gate1'
        // is tagged and perfectly valid; the top field simply was not written,
        // so it reads as zero. That has to come out as the two knobs it saved
        // and NO height gate -- read as garbage, or refused for not carrying a
        // third value, a gun that had a working shape gate either boots with a
        // height cut nobody set or loses the gate it did set.
        nvs_set_u32(1, "gate1", 0x6A000000u | (12u << 6) | 20u);
        gp = ga = gb = -1;
        ck(aim_gate2_load(&gp, &ga, &gb) && gp == 12 && ga == 20,
           "an old two-field word still loads its pxmax and its armax");
        ck(gb == 0,
           "...with bhmax reading 0 -- off, which is what a gun that never had "
           "the knob was running, and not whatever the bits above the old "
           "payload happen to hold");
        ck(aim_gate2_clear(), "clearing it succeeds");
        ck(!aim_gate2_load(&gp, &ga, &gb), "after which nothing is stored");
        ck(aim_gate2_clear(),
           "and clearing an ALREADY absent key is not a failure -- camreset "
           "erases both gates on every gun it runs on, including the ones that "
           "have neither, and it must not report that it could not");

        // ---- the two keys do not touch each other --------------------------
        int sf = -1, smn = -1, smx = -1, srt = -1;
        ck(aim_gate_store(1, 4, 11, 6) && aim_gate2_store(12, 20, 34),
           "a gun with both gates saved");
        ck(aim_gate_load(&sf, &smn, &smx, &srt)
           && sf == 1 && smn == 4 && smx == 11 && srt == 6,
           "the size window reads back its own four fields with a shape gate "
           "written after it");
        ck(aim_gate2_load(&gp, &ga, &gb) && gp == 12 && ga == 20 && gb == 34,
           "and the shape gate its own three -- one key never answers for the "
           "other, which sharing a key is precisely how they would");
        ck(aim_gate2_clear(), "clear the shape gate on its own");
        ck(!aim_gate2_load(&gp, &ga, &gb), "...it is gone");
        sf = smn = smx = srt = -1;
        ck(aim_gate_load(&sf, &smn, &smx, &srt)
           && sf == 1 && smn == 4 && smx == 11 && srt == 6,
           "...and the size window is untouched: erasing one gate must not "
           "take the other down with it");
        ck(aim_gate2_store(12, 20, 34) && aim_gate_clear(),
           "now the other way up");
        ck(!aim_gate_load(&sf, &smn, &smx, &srt), "the size window is gone");
        ck(aim_gate2_load(&gp, &ga, &gb) && gp == 12 && ga == 20 && gb == 34,
           "...and the shape gate outlives it");
        // Corruption travels no further than its own key either. A word that
        // fails its tag check is one unreadable setting, not two.
        ck(aim_gate_store(1, 4, 11, 6), "both stored again");
        nvs_set_u32(1, "gate0", 0x00001234u);
        ck(!aim_gate_load(&sf, &smn, &smx, &srt),
           "a corrupt size-gate word reads as nothing stored");
        ck(aim_gate2_load(&gp, &ga, &gb) && gp == 12 && ga == 20 && gb == 34,
           "...and the shape gate beside it is still perfectly readable");
        ck(aim_gate_store(1, 4, 11, 6), "repair the size gate");
        nvs_set_u32(1, "gate1", 0x00005678u);
        ck(!aim_gate2_load(&gp, &ga, &gb),
           "and a corrupt shape-gate word reads as nothing stored too");
        sf = smn = smx = srt = -1;
        ck(aim_gate_load(&sf, &smn, &smx, &srt) && smn == 4 && smx == 11,
           "...without stopping the size window loading, which is the whole "
           "point of giving it a key of its own");

        // ---- a gun flashed before the shape gate existed ---------------------
        // It has a 'gate0' and no 'gate1' at all, and it has to come up with
        // its size window exactly as it saved it and the shape gate OFF. Read
        // out of the other key's bits, the perfectly ordinary saved gate
        // fmt:1 bmin:4 bmax:11 rtol:6 spells pxmax=18 armax=54 -- a shape gate
        // on a gun that has never had one, dropping blobs nobody asked it to.
        wiicam_cam_command("cam=bhmax:0,pxmax:0,armax:0");   // as the statics ship
        ck(aim_gate_store(1, 4, 11, 6), "an old gun's saved size window");
        ck(aim_gate2_clear(), "...and nothing at all under the second key");
        wiicam_aim_begin();
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("fmt=1") != std::string::npos
           && g_replies[0].find("bmin=4") != std::string::npos
           && g_replies[0].find("bmax=11") != std::string::npos
           && g_replies[0].find("rtol=6") != std::string::npos,
           "it boots with its size window exactly as it was saved");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=0 ") != std::string::npos
           && g_replies[0].find("pxmax=0 ") != std::string::npos
           && g_replies[0].find("armax=0 ") != std::string::npos,
           "...and the shape gate off, height cut included: an absent 'gate1' "
           "means no shape gate, never a gate of zeroes and never one read out "
           "of gate0");

        // ---- end to end through the serial surface ---------------------------
        wiicam_cam_command("cam=bhmax:10,pxmax:14,armax:20");
        g_replies.clear();
        wiicam_cam_command("camsave");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=10 pxmax=14 armax=20")
              != std::string::npos,
           "camsave names all three shape knobs it wrote alongside the size "
           "one, bhmax immediately before pxmax as everywhere else, so a tool "
           "can VERIFY what landed rather than assume it");
        ck(aim_gate2_load(&gp, &ga, &gb) && gp == 14 && ga == 20 && gb == 10,
           "...and the second word really holds all three knobs -- the reply "
           "is not the thing that decides what the next boot does");
        {
            int cf = -1, cmn = -1, cmx = -1, crt = -1;
            ck(aim_gate_load(&cf, &cmn, &cmx, &crt) && cmn == 4 && cmx == 11,
               "...and the size window it was saved beside is still there, "
               "written to its own key in the same camsave");
        }
        // Forget the live copy. bhmax goes to a DIFFERENT wrong value from the
        // other two, so a reboot that restored the three fields in the wrong
        // order could not be mistaken for one that restored them at all.
        wiicam_cam_command("cam=bhmax:0,pxmax:0,armax:0");
        wiicam_aim_begin();
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=10 pxmax=14 armax=20 ")
              != std::string::npos,
           "a reboot brings the shape gate back with all three knobs in their "
           "own slots, which is the only reason it is worth saving and also "
           "the only reason camreset has to erase it");
        g_replies.clear();
        wiicam_cam_command("camreset");
        ck(!aim_gate2_load(&gp, &ga, &gb),
           "camreset erases the saved shape gate too: a gate that stops the "
           "gun aiming would otherwise come back on the next boot and the one "
           "command a user reaches for when nothing works would fix the "
           "session and lose the argument");
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=0 ") != std::string::npos
           && g_replies[0].find("pxmax=0 ") != std::string::npos
           && g_replies[0].find("armax=0 ") != std::string::npos,
           "...and the LIVE knobs go to 0 with it, the height cut included, so "
           "the recovery command does not leave the gate running until the "
           "next power cycle");
        wiicam_aim_begin();
        g_replies.clear();
        wiicam_cam_command("cam?");
        ck(!g_replies.empty()
           && g_replies[0].find("bhmax=0 ") != std::string::npos
           && g_replies[0].find("pxmax=0 ") != std::string::npos
           && g_replies[0].find("armax=0 ") != std::string::npos,
           "so the boot after that one comes up with no shape gate either");
    }

    printf("\nwiicam adapter: %s (%d failures)\n", fails ? "FAILED" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
