// The RP2040/wiicam front end, on the host. The one property that must never
// regress: an unseen slot's STALE coordinates (which the DFRobot driver
// retains forever) must not reach the resolver. Plus: normalisation into
// 240x176, raw mode for lens sweeps, lock + solve through a real calibration,
// lead extrapolation, and the ~cam command subset.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
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
        if(*l<g_ll) return -1; memcpy(o,g_lens,g_ll); *l=g_ll; return ESP_OK; }
    if(!g_bh) return ESP_ERR_NVS_NOT_FOUND;
    if(*l<g_bl) return -1; memcpy(o,g_blob,g_bl); *l=g_bl; return ESP_OK; }
esp_err_t nvs_erase_key(nvs_handle_t, const char* k){ if(is_lens(k)) g_lh=false; else g_bh=false; return ESP_OK; }
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
static uint32_t g_u32=0; static bool g_uh=false;
esp_err_t nvs_set_u32(nvs_handle_t, const char*, uint32_t v){ g_u32=v; g_uh=true; return ESP_OK; }
esp_err_t nvs_get_u32(nvs_handle_t, const char*, uint32_t* v){ if(!g_uh) return ESP_ERR_NVS_NOT_FOUND; *v=g_u32; return ESP_OK; }
esp_err_t nvs_commit(nvs_handle_t){ return ESP_OK; }
void nvs_close(nvs_handle_t){}
esp_err_t nvs_flash_init(void){ return ESP_OK; }
int64_t esp_timer_get_time(void){ return 0; }
}

static std::vector<std::string> g_lines, g_replies;
static void line_sink(const char* s){ g_lines.push_back(s); }
static void reply_sink(const char* s){ g_replies.push_back(s); }

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
    float base = -1.0f;
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
           && g_replies[0].find("ext=0") != std::string::npos
           && g_replies[0].find("bmin=0") != std::string::npos
           && g_replies[0].find("bmax=15") != std::string::npos,
           "the gate ships inert: extended format off, window wide open");

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

        // The safety valve: a gate set so tight that nothing survives passes
        // the frame through untouched. A blinded gun is worse than an impostor.
        wiicam_cam_command("cam=bmin:12,bmax:13");
        g_lines.clear();
        t += DT;
        px[0] += 1;
        wiicam_aim_process_sz(px, py, sz, 0xF, t, &sx, &sy);
        n = -1;
        if (!g_lines.empty()) sscanf(g_lines[0].c_str(), "Q,%lu,%d", &ms, &n);
        ck(n == 4, "a gate that would reject EVERY blob is ignored for that "
                   "frame -- it can never blind the gun");

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
        wiicam_cam_command("cam=ext:1");
        const int ep = wiicam_aim_ext_epoch();
        ck(wiicam_aim_ext() == 1, "ext:1 asks for the extended data format");
        ck(ep != 0, "and bumps the epoch so the camera owner re-applies it");
        wiicam_cam_command("cam=ext:1");
        ck(wiicam_aim_ext_epoch() == ep,
           "setting it again changes nothing -- no needless I2C writes");
        wiicam_aim_format_dirty();
        ck(wiicam_aim_ext_epoch() != ep,
           "a camera restart bumps it: the format must be applied again or "
           "the frames decode as garbage with no error at all");
        ck(wiicam_aim_ext() == 1,
           "and the wanted format survives that bump");
        // The camera poll reads ONE word. Reading the flag and the epoch as
        // two values let it pair a new epoch with the old flag, write the old
        // format, latch the new epoch, and then never correct itself.
        ck((wiicam_aim_ext_state() & 1) == wiicam_aim_ext()
           && (wiicam_aim_ext_state() >> 1) == wiicam_aim_ext_epoch(),
           "the state word carries both halves, so they cannot be read apart");
        {
            const int st = wiicam_aim_ext_state();
            wiicam_cam_command("cam=ext:0");
            ck(wiicam_aim_ext_state() != st,
               "and every change moves the whole word at once");
        }
        wiicam_cam_command("cam=ext:1");
        g_replies.clear();
        wiicam_cam_command("camreset");
        ck(wiicam_aim_ext() == 0, "camreset turns the extended format back off");
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

    printf("\nwiicam adapter: %s (%d failures)\n", fails ? "FAILED" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
