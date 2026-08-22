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
// the lead is an i16 under its own key, deliberately NOT part of the cam blob
static int16_t g_i16 = 0; static bool g_i16have = false;
esp_err_t nvs_set_i16(nvs_handle_t, const char*, int16_t v) {
    g_i16 = v; g_i16have = true; return ESP_OK;
}
esp_err_t nvs_get_i16(nvs_handle_t, const char*, int16_t* v) {
    if (!g_i16have) return ESP_ERR_NVS_NOT_FOUND;
    *v = g_i16;
    return ESP_OK;
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
    aim_lens_t fisheye = { 2, 0.0f, 0.0f, 84.0f, 85.9f };
    ck(aim_lens_store(&fisheye), "store a fisheye setup");
    ck(aim_lens_load(&ln) && ln.model==2 && ln.feq==85.9f && ln.fpx==84.0f,
       "loads back identically");
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
