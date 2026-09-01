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
static bool is_gate(const char* k){ return k && !strcmp(k, "gate0"); }
static uint32_t g_u32=0; static bool g_uh=false;
esp_err_t nvs_erase_key(nvs_handle_t, const char* k){
    if(is_gate(k)) g_uh=false; else if(is_lens(k)) g_lh=false; else g_bh=false;
    return ESP_OK; }
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
esp_err_t nvs_set_u32(nvs_handle_t, const char*, uint32_t v){ g_u32=v; g_uh=true; return ESP_OK; }
esp_err_t nvs_get_u32(nvs_handle_t, const char*, uint32_t* v){ if(!g_uh) return ESP_ERR_NVS_NOT_FOUND; *v=g_u32; return ESP_OK; }
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

        // The box and the intensity are REPORTED, not gated: a discriminator
        // invented before anyone has seen a real number out of it is the same
        // guess the size window would have been. So '~camblob?' is the only
        // way they reach anyone, and in full mode each tuple grows by three.
        ck(wiicam_aim_full_poll(fpx, fpy, fsz, &fseen) == 1, "a clean frame again");
        wiicam_cam_command("cam=fmt:2");
        t += DT;
        wiicam_aim_process_sz(fpx, fpy, fsz, fseen, t, &sx, &sy);
        g_replies.clear();
        wiicam_cam_command("camblob?");
        int f[21];
        int got = 0;
        if (g_replies.size() >= 2)
            got = sscanf(g_replies[1].c_str(),
                         "CAM: blobs %d,%d,%d,%d,%d,%d,%d"
                         " %d,%d,%d,%d,%d,%d,%d"
                         " %d,%d,%d,%d,%d,%d,%d",
                         &f[0],&f[1],&f[2],&f[3],&f[4],&f[5],&f[6],
                         &f[7],&f[8],&f[9],&f[10],&f[11],&f[12],&f[13],
                         &f[14],&f[15],&f[16],&f[17],&f[18],&f[19],&f[20]);
        ck(got == 21,
           "in full mode every blob tuple carries SEVEN fields: the four the "
           "other formats have, plus box width, box height and intensity");
        ck(got == 21 && f[2] == 3 && f[3] == 1,
           "the first four are still position, size and the gate's verdict, "
           "in that order -- the extras are appended, not inserted");
        ck(got == 21 && f[4] == 8 && f[5] == 12 && f[6] == 200,
           "object 0's box is 8x12 at intensity 200: its xMax arrived as 0x92 "
           "and the corners are SEVEN bit, so read as eight it would be a "
           "136-wide box on a blob three pixels across");
        ck(got == 21 && f[11] == 4 && f[12] == 7 && f[13] == 255,
           "object 1's yMin arrived as 0x82; masked to 2 it still gives a "
           "positive height instead of collapsing to zero");
        ck(got == 21 && f[18] == 0 && f[19] == 3 && f[20] == 1,
           "and a box whose xMax is BELOW its xMin reads as zero width, not "
           "as the 252 an unsigned subtraction wraps to");

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

        // ---- the box must travel with its own blob -------------------------
        // The poll fills its box arrays by HARDWARE SLOT, 0..3 as the sensor
        // numbers them. Everything else in the report -- position, size, kept
        // -- is indexed by the blob's place in the COMPACTED seen list,
        // because process_sz skips empty slots as it walks. Publish the box
        // straight from the slot-indexed array and the two agree only while
        // the empty slot is the LAST one: any other gap and blob 0 is printed
        // with slot 0's box, which belongs to no blob on screen and may be
        // left over from an earlier frame entirely.
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
        int fb[28];
        int gotb = 0;

        // Slot ZERO empty. Every box and intensity below is distinct, so a
        // tuple wearing the wrong one says which slot it was taken from.
        static const FullObj GAP0[4] = {
        //    x     y  sz  xmn  ymn  xmx  ymx  inten
            { 1023,1023, 15,  0,   0,   0,   0,   0 },  // EMPTY
            {  100, 200,  3, 10,  20,  18,  32, 111 },  // box  8x12
            {  300, 250,  4, 10,  20,  30,  41, 122 },  // box 20x21
            {  500, 300,  5, 10,  20,  50,  61, 133 },  // box 40x41
        };
        {
            const std::string ln = full_report(GAP0, 0xEu);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],
                          &fb[7],&fb[8],&fb[9],&fb[10],&fb[11],&fb[12],&fb[13],
                          &fb[14],&fb[15],&fb[16],&fb[17],&fb[18],&fb[19],&fb[20]);
            ck(qseen == 0xEu && gotb == 21,
               "slot 0 empty: the report lists the three blobs that are there");
            ck(gotb == 21 && fb[2] == 3 && fb[4] == 8 && fb[5] == 12
               && fb[6] == 111,
               "and the FIRST tuple carries slot 1's box, because slot 1 is "
               "the first blob -- slot 0's box belongs to no blob on screen");
            ck(gotb == 21 && fb[9] == 4 && fb[11] == 20 && fb[12] == 21
               && fb[13] == 122,
               "the second carries slot 2's, not slot 1's -- every entry after "
               "a gap is off by one the moment the box is read by slot");
            ck(gotb == 21 && fb[16] == 5 && fb[18] == 40 && fb[19] == 41
               && fb[20] == 133,
               "and the third slot 3's, so the last real blob's box is printed "
               "at all rather than falling off the end of the list");
        }

        // A gap in the MIDDLE shifts only the tail, so the first entries look
        // right and only the ones after the gap are wrong -- the version of
        // this bug that survives a glance at the readout.
        static const FullObj GAP2[4] = {
        //    x     y  sz  xmn  ymn  xmx  ymx  inten
            {  110, 210,  6,  1,   2,  10,  15, 144 },  // box  9x13
            {  310, 260,  7,  1,   2,  23,  25, 155 },  // box 22x23
            { 1023,1023, 15,  0,   0,   0,   0,   0 },  // EMPTY
            {  510, 310,  8,  1,   2,  45,  47, 166 },  // box 44x45
        };
        {
            const std::string ln = full_report(GAP2, 0xBu);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],
                          &fb[7],&fb[8],&fb[9],&fb[10],&fb[11],&fb[12],&fb[13],
                          &fb[14],&fb[15],&fb[16],&fb[17],&fb[18],&fb[19],&fb[20]);
            ck(qseen == 0xBu && gotb == 21, "slot 2 empty: three blobs again");
            ck(gotb == 21 && fb[4] == 9 && fb[5] == 13 && fb[6] == 144
               && fb[11] == 22 && fb[12] == 23 && fb[13] == 155,
               "the two blobs BEFORE the gap keep their own boxes -- this is "
               "the shape of the bug that reads as fine until the tail");
            ck(gotb == 21 && fb[16] == 8 && fb[18] == 44 && fb[19] == 45
               && fb[20] == 166,
               "and the one after it carries slot 3's box, not the empty "
               "slot 2's zeroes");
        }

        // Leaving full mode. The box columns stop being printed, and there is
        // nowhere for the last full frame's numbers to hide in a four-field
        // tuple -- so count the commas rather than trusting the first one.
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
               "nothing more: nine commas, not eighteen");
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
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],
                          &fb[7],&fb[8],&fb[9],&fb[10],&fb[11],&fb[12],&fb[13],
                          &fb[14],&fb[15],&fb[16],&fb[17],&fb[18],&fb[19],&fb[20]);
        ck(gotb == 21 && fb[4] == 0 && fb[5] == 0 && fb[6] == 0
           && fb[11] == 0 && fb[12] == 0 && fb[13] == 0
           && fb[18] == 0 && fb[19] == 0 && fb[20] == 0,
           "a format round trip through extended leaves every box at zero "
           "until a full frame is read again");

        // An unseen slot reads as NO BOX, never as the box it had last frame.
        static const FullObj FOUR[4] = {
        //    x     y  sz  xmn  ymn  xmx  ymx  inten
            { 120, 220,  2,  1,   2,  12,  15,  77 },   // box 11x13
            { 320, 270,  6,  1,   2,  27,  35,  88 },   // box 26x33
            { 520, 320,  9,  1,   2,  61,  66,  99 },   // box 60x64
            { 720, 370, 11,  1,   2,  71,  76, 210 },   // box 70x74
        };
        static const FullObj THREE[4] = {
            { 120, 220,  2,  1,   2,  12,  15,  77 },
            { 320, 270,  6,  1,   2,  27,  35,  88 },
            { 520, 320,  9,  1,   2,  61,  66,  99 },
            { 1023,1023, 15, 0,   0,   0,   0,   0 },   // slot 3 goes dark
        };
        {
            const std::string ln = full_report(FOUR, 0xFu);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],
                          &fb[7],&fb[8],&fb[9],&fb[10],&fb[11],&fb[12],&fb[13],
                          &fb[14],&fb[15],&fb[16],&fb[17],&fb[18],&fb[19],&fb[20],
                          &fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26],&fb[27]);
            ck(qseen == 0xFu && gotb == 28,
               "with no gap at all, four tuples and four boxes");
            ck(gotb == 28 && fb[4] == 11 && fb[11] == 26 && fb[18] == 60
               && fb[25] == 70,
               "each the box of the blob it is printed beside -- the case "
               "where slot order and report order happen to be the same");
        }
        {
            // Same three blobs, slot 3 dark. Nothing of slot 3's 70x74 at 210
            // may appear anywhere in this frame's report.
            const std::string ln = full_report(THREE, 0x7u);
            gotb = sscanf(ln.c_str(),
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d %d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],
                          &fb[7],&fb[8],&fb[9],&fb[10],&fb[11],&fb[12],&fb[13],
                          &fb[14],&fb[15],&fb[16],&fb[17],&fb[18],&fb[19],&fb[20],
                          &fb[21]);
            ck(qseen == 0x7u && gotb == 21,
               "the slot that went dark is dropped from the list, and no "
               "fourth tuple is printed from the frame before it");
            ck(gotb == 21 && fb[4] == 11 && fb[11] == 26 && fb[18] == 60,
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
                          "CAM: blobs %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d"
                          " %d,%d,%d,%d,%d,%d,%d",
                          &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],
                          &fb[7],&fb[8],&fb[9],&fb[10],&fb[11],&fb[12],&fb[13],
                          &fb[14],&fb[15],&fb[16],&fb[17],&fb[18],&fb[19],&fb[20],
                          &fb[21],&fb[22],&fb[23],&fb[24],&fb[25],&fb[26],&fb[27]);
            ck(gotb == 28 && fb[25] == 0 && fb[26] == 0 && fb[27] == 0,
               "an unseen slot reads as no box at all -- 0x0 at intensity 0 "
               "-- rather than as the box it measured on the frame before");
            ck(gotb == 28 && fb[4] == 11 && fb[11] == 26 && fb[18] == 60,
               "and the blobs that ARE there are untouched by that");
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
                              "CAM: blobs %d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d"
                              " %d,%d,%d,%d,%d,%d,%d %d",
                              &fb[0],&fb[1],&fb[2],&fb[3],&fb[4],&fb[5],&fb[6],
                              &fb[7],&fb[8],&fb[9],&fb[10],&fb[11],&fb[12],&fb[13],
                              &fb[14],&fb[15],&fb[16],&fb[17],&fb[18],&fb[19],&fb[20],
                              &fb[21]);
            }
            ck(rbn == 3 && gotb == 21,
               "a frame landing mid-reply changes neither line: still three "
               "counted and three listed, not four");
            ck(gotb == 21 && fb[2] == 3 && fb[4] == 8
               && fb[9] == 4 && fb[11] == 20
               && fb[16] == 5 && fb[18] == 40,
               "and they are the SAME three -- sizes and boxes from the frame "
               "line 1 counted, not the one that arrived between the lines");
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

    printf("\nwiicam adapter: %s (%d failures)\n", fails ? "FAILED" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
