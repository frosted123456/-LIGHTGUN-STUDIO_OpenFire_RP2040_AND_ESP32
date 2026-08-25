// Compiles AND LINKS the ESP_PLATFORM branch of aim_runtime.cpp against the fake
// ESP-IDF headers in fakeinc/, so a linkage mistake (C vs C++ mangling) fails
// here rather than on the device. Covers NVS round trips for the calibration,
// camera, lead, lens and boot-counter keys, and their independence.
#include <stdio.h>
#include <string.h>
#include "aim_runtime.h"
#include "nvs.h"
#include "esp_timer.h"

// --- fake ESP world -------------------------------------------------------
static int64_t g_now = 0;
extern "C" int64_t esp_timer_get_time(void) { return g_now; }

static unsigned char g_blob[512];
static size_t        g_blob_len = 0;
static bool          g_have = false;
// a second key, so the calibration and the camera settings can be shown to be
// independent -- clearing one must not disturb the other
static unsigned char g_cam[128];
static size_t        g_camlen = 0;
static bool          g_camhave = false;
static bool key_is_cam(const char* k){ return k && k[0]=='c' && k[1]=='a'; }
// a third slot for the lens blob -- "lens0" must not alias the calibration's
// "c0", or a lens store would clobber the calibration in this fake and the
// independence checks below would test nothing
static unsigned char g_lens[64];
static size_t        g_lenslen = 0;
static bool          g_lenshave = false;
static bool key_is_lens(const char* k){ return k && k[0]=='l' && k[1]=='e' && k[2]=='n'; }

extern "C" {
esp_err_t nvs_open(const char*, nvs_open_mode_t, nvs_handle_t* h) { *h = 1; return ESP_OK; }
esp_err_t nvs_set_blob(nvs_handle_t, const char* k, const void* v, size_t n) {
    if (key_is_cam(k)) { if (n>sizeof(g_cam)) return -1;
        memcpy(g_cam,v,n); g_camlen=n; g_camhave=true; return ESP_OK; }
    if (key_is_lens(k)) { if (n>sizeof(g_lens)) return -1;
        memcpy(g_lens,v,n); g_lenslen=n; g_lenshave=true; return ESP_OK; }
    if (n > sizeof(g_blob)) return -1;
    memcpy(g_blob, v, n); g_blob_len = n; g_have = true; return ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t, const char* k, void* out, size_t* len) {
    if (key_is_cam(k)) { if(!g_camhave) return ESP_ERR_NVS_NOT_FOUND;
        if(*len<g_camlen) return -1; memcpy(out,g_cam,g_camlen); *len=g_camlen; return ESP_OK; }
    if (key_is_lens(k)) { if(!g_lenshave) return ESP_ERR_NVS_NOT_FOUND;
        if(*len<g_lenslen) return -1; memcpy(out,g_lens,g_lenslen); *len=g_lenslen; return ESP_OK; }
    if (!g_have) return ESP_ERR_NVS_NOT_FOUND;
    if (*len < g_blob_len) return -1;
    memcpy(out, g_blob, g_blob_len); *len = g_blob_len; return ESP_OK;
}
esp_err_t nvs_erase_key(nvs_handle_t, const char* k) {
    if (key_is_cam(k)) g_camhave = false;
    else if (key_is_lens(k)) g_lenshave = false;
    else g_have = false; return ESP_OK; }
// the boot counter, a u32 under its own key
static uint32_t g_u32 = 0; static bool g_u32have = false;
esp_err_t nvs_set_u32(nvs_handle_t, const char*, uint32_t v) {
    g_u32 = v; g_u32have = true; return ESP_OK;
}
esp_err_t nvs_get_u32(nvs_handle_t, const char*, uint32_t* v) {
    if (!g_u32have) return ESP_ERR_NVS_NOT_FOUND;
    *v = g_u32;
    return ESP_OK;
}
// i16 values live under their own keys (lead, smoothing); the stub is
// key-aware or the second key would silently alias the first.
struct I16Slot { char key[16]; int16_t v; bool have; };
static I16Slot g_i16s[8];   // lead0 smth0 dead0 tmod0 fir0, plus room
esp_err_t nvs_set_i16(nvs_handle_t, const char* k, int16_t v) {
    for (auto& s : g_i16s)
        if (s.have && !strcmp(s.key, k)) { s.v = v; return ESP_OK; }
    for (auto& s : g_i16s)
        if (!s.have) { strncpy(s.key, k, 15); s.v = v; s.have = true; return ESP_OK; }
    return -1;
}
esp_err_t nvs_get_i16(nvs_handle_t, const char* k, int16_t* v) {
    for (auto& s : g_i16s)
        if (s.have && !strcmp(s.key, k)) { *v = s.v; return ESP_OK; }
    return ESP_ERR_NVS_NOT_FOUND;
}
esp_err_t nvs_commit(nvs_handle_t) { return ESP_OK; }
void      nvs_close(nvs_handle_t) {}
esp_err_t nvs_flash_init(void) { return ESP_OK; }
}

static int fails = 0;
static void ck(bool ok, const char* m) { printf("  [%s] %s\n", ok?"PASS":"FAIL", m); if(!ok) fails++; }

int main()
{
    printf("ESP_PLATFORM branch: compile, link and NVS round trip\n\n");
    aim_runtime_begin();
    ck(!aim_runtime_active(), "empty store -> inactive");

    ck(aim_runtime_command("aimcal=0.5046,0.2988,0.3513,1.2003,4.8,-2.12"),
       "install (writes the fake NVS)");
    ck(g_have && g_blob_len == sizeof(aim_calib_t), "blob stored at the right size");

    // simulate a reboot: wipe RAM state, reload from the store
    aim_runtime_enable(false);
    aim_runtime_begin();
    ck(aim_runtime_active(), "survives a reboot (loaded back from NVS)");
    const aim_calib_t* c = aim_runtime_calib();
    ck(c->bx == 4.8f && c->w == 0.3513f, "values identical after reload");

    printf("\ntrigger markers (this is the code that failed to link):\n");
    g_now = 1234567;                      // 1234 ms
    aim_runtime_command("aimcap=1");
    ck(aim_runtime_capture_on(), "aimcap=1 enables markers");
    printf("       expect one T,1234 line next ->\n       ");
    aim_runtime_trigger_tick(true);       // press edge: emits
    aim_runtime_trigger_tick(true);       // held: must NOT re-emit
    aim_runtime_trigger_tick(false);
    printf("       (exactly one line above = edge detection works)\n");
    aim_runtime_command("aimcap=0");
    aim_runtime_trigger_tick(true);
    ck(!aim_runtime_capture_on(), "aimcap=0 silences them");

    printf("\nlatency lead persistence:\n");
    int lead = -1;
    ck(!aim_lead_load(&lead), "nothing stored to begin with");
    ck(aim_lead_store(12), "store 12 ms");
    ck(aim_lead_load(&lead) && lead == 12, "reads back 12 ms");
    ck(aim_lead_store(999) && aim_lead_load(&lead) && lead == 50,
       "clamped to the same 50 ms ceiling the capture layer enforces");
    ck(aim_lead_store(-5) && aim_lead_load(&lead) && lead == 0, "negative clamped to 0");
    // and it must be INDEPENDENT of the camera blob, which is the whole reason
    // it is not a fifth field in aim_cam_t
    aim_cam_t cam = { 60, 40, 8, 1 };
    ck(aim_cam_store(&cam), "store camera settings");
    aim_lead_store(7);
    aim_cam_t back0;
    ck(aim_cam_load(&back0) && back0.thr == 60 && back0.aec == 40,
       "camera settings untouched by a lead write");
    ck(aim_lead_load(&lead) && lead == 7, "lead untouched by a camera write");

    printf("\nsmoothing level:\n");
    int sm = -1;
    ck(!aim_smooth_load(&sm), "nothing stored to begin with");
    ck(aim_smooth_store(7), "store level 7");
    ck(aim_smooth_load(&sm) && sm == 7, "reads back 7");
    ck(aim_smooth_store(99) && aim_smooth_load(&sm) && sm == 10, "clamped to 10");
    ck(aim_smooth_store(-2) && aim_smooth_load(&sm) && sm == 0, "clamped to 0");
    aim_smooth_set(3);
    const float fc3 = aim_filter_min_cutoff(), b3 = aim_filter_beta();
    ck(fc3 == 3.5f && b3 == 15.0f, "level 3 is the build-time default pair");
    aim_smooth_set(10);
    ck(aim_filter_min_cutoff() < fc3 && aim_filter_min_cutoff() == 1.0f,
       "level 10 is heavier than 3 and equals the pre-retune default");
    // table sanity: strictly more smoothing per level, motion never filtered
    bool mono = true;
    float prev_fc = 1e9f;
    for (int l = 1; l <= 10; ++l) {
        aim_smooth_set(l);
        mono &= aim_filter_min_cutoff() < prev_fc && aim_filter_min_cutoff() > 0.0f
             && aim_filter_beta() >= 15.0f;
        prev_fc = aim_filter_min_cutoff();
    }
    ck(mono, "levels 1..10 are monotone, non-zero, beta stays >= 15");
    aim_smooth_set(0);
    ck(aim_filter_min_cutoff() == 0.0f, "level 0 turns the filter off");
    ck(aim_smooth_get() == 0, "get reports the level that was set");
    aim_smooth_set(3);                        // back to defaults for the rest
    // lead and smoothing are both i16 keys; they must not alias
    aim_lead_store(12); aim_smooth_store(5);
    ck(aim_lead_load(&lead) && lead == 12 && aim_smooth_load(&sm) && sm == 5,
       "lead and smoothing keys do not alias each other");

    printf("\noutput dead-band:\n");
    int dd = -1;
    ck(!aim_dead_load(&dd), "nothing stored to begin with");
    ck(aim_dead_store(24) && aim_dead_load(&dd) && dd == 24, "store + read back 24");
    ck(aim_dead_store(-3) && aim_dead_load(&dd) && dd == 0, "negative clamped to 0");
    aim_dead_set(0);
    ck(aim_dead_pass(100, 100) && aim_dead_pass(101, 100),
       "threshold 0 passes everything");
    aim_dead_set(20);
    ck(aim_dead_pass(500, 500), "first position after a jump passes");
    ck(!aim_dead_pass(510, 500), "10 units of shimmer is swallowed");
    ck(!aim_dead_pass(500, 514), "14 units the other way is swallowed too");
    ck(aim_dead_pass(521, 500), "21 units is real motion and passes");
    ck(!aim_dead_pass(530, 500),
       "the reference moved to the SENT position, so 9 more is shimmer");
    ck(aim_dead_pass(600, 600) && aim_dead_pass(700, 700),
       "supra-threshold motion streams through with no added delay");
    aim_dead_set(0);
    // smoothing and dead-band keys must not alias
    aim_smooth_store(5); aim_dead_store(30);
    ck(aim_smooth_load(&sm) && sm == 5 && aim_dead_load(&dd) && dd == 30,
       "smoothing and dead-band keys do not alias");

    // a stored blob of the wrong size must be rejected, not reinterpreted
    g_blob_len = sizeof(aim_calib_t) - 4;
    aim_runtime_begin();
    ck(!aim_runtime_active(), "wrong-sized stored blob rejected, not reinterpreted");

    printf("\ncamera settings persistence:\n");
    aim_cam_t cs = { 72, 55, 3, 1 };
    ck(aim_cam_store(&cs), "store accepted");
    aim_cam_t back = {};
    ck(aim_cam_load(&back), "loads back");
    ck(back.thr==72 && back.aec==55 && back.agc==3 && back.boost==1, "values identical");
    // out-of-range must be refused, not written -- a stored blob must never be
    // able to push the sensor somewhere the live console would reject
    aim_cam_t bad = { 2, 55, 3, 0 };
    ck(!aim_cam_store(&bad), "thr below the console's own minimum refused");
    bad = { 72, 55, 99, 0 };
    ck(!aim_cam_store(&bad), "agc above 30 refused");
    ck(aim_cam_load(&back) && back.thr==72, "a refused store left the good one intact");
    // independence from the calibration
    aim_runtime_command("aimcal=0.5046,0.2988,0.3513,1.2003,4.8,-2.12");
    aim_cam_clear();
    ck(!aim_cam_load(&back), "camreset forgot the camera settings");
    ck(aim_runtime_active(), "...and left the CALIBRATION untouched");

    printf("\nlens correction persistence:\n");
    aim_lens_t ln = {};
    ck(!aim_lens_load(&ln), "nothing stored to begin with");
    aim_lens_t fisheye = { 2, 0.0f, 0.0f, 84.0f, 85.9f, 3.2f, -1.5f };
    ck(aim_lens_store(&fisheye), "store a fisheye setup");
    ck(aim_lens_load(&ln) && ln.model==2 && ln.feq==85.9f && ln.fpx==84.0f
       && ln.cx==3.2f && ln.cy==-1.5f,
       "loads back identically, distortion centre included");
    aim_lens_t badcen = { 2, 0.0f, 0.0f, 84.0f, 85.9f, 500.0f, 0.0f };
    ck(!aim_lens_store(&badcen), "absurd distortion centre refused");
    aim_lens_t badl = { 3, 0, 0, 84.0f, 85.9f };
    ck(!aim_lens_store(&badl), "unknown model refused");
    badl = { 1, 5.0f, 0, 184.7f, 90.0f };
    ck(!aim_lens_store(&badl), "absurd k1 refused");
    badl = { 1, 0.0f/0.0f, 0, 184.7f, 90.0f };
    ck(!aim_lens_store(&badl), "NaN k1 refused (the NaN-safe comparison works)");
    ck(aim_lens_load(&ln) && ln.model==2, "refused stores left the good one intact");
    // independence, both ways
    ck(aim_runtime_active(), "calibration still active after lens writes");
    aim_lens_clear();
    ck(!aim_lens_load(&ln), "lens cleared");
    ck(aim_runtime_active(), "...calibration still untouched");

    printf("\nboot forensics:\n");
    const uint32_t b0 = aim_boot_count();
    aim_runtime_begin();
    ck(aim_boot_count() == b0 + 1, "boot counter increments on every begin");
    aim_runtime_begin();
    ck(aim_boot_count() == b0 + 2, "and again (persisted through the fake NVS)");
    ck(aim_reset_reason() != 0 && aim_reset_reason()[0] != 0, "reset reason names something");

    printf("\n%s (%d failures)\n", fails?"FAILURES":"ALL PASS", fails);
    return fails ? 1 : 0;
}
