// LittleFS-backed shim implementation. One file per key under /aim_<key>.
#ifdef ARDUINO_ARCH_RP2040
#include "nvs.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <pico/time.h>
#include <hardware/watchdog.h>
#include "esp_system.h"

static void key_path(const char* key, char* out, size_t n)
{
    snprintf(out, n, "/aim_%s", key ? key : "nil");
}

extern "C" {

esp_err_t nvs_open(const char*, nvs_open_mode_t, nvs_handle_t* h)
{
    if (!LittleFS.begin()) return -1;
    if (h) *h = 1;
    return ESP_OK;
}

esp_err_t nvs_set_blob(nvs_handle_t, const char* key, const void* v, size_t n)
{
    char p[32]; key_path(key, p, sizeof(p));
    File f = LittleFS.open(p, "w");
    if (!f) return -1;
    const size_t w = f.write((const uint8_t*)v, n);
    f.close();
    return (w == n) ? ESP_OK : -1;
}

esp_err_t nvs_get_blob(nvs_handle_t, const char* key, void* out, size_t* len)
{
    char p[32]; key_path(key, p, sizeof(p));
    File f = LittleFS.open(p, "r");
    if (!f) return ESP_ERR_NVS_NOT_FOUND;
    const size_t sz = f.size();
    if (sz > *len) { f.close(); return -1; }
    const size_t r = f.read((uint8_t*)out, sz);
    f.close();
    if (r != sz) return -1;
    *len = sz;
    return ESP_OK;
}

esp_err_t nvs_erase_key(nvs_handle_t, const char* key)
{
    char p[32]; key_path(key, p, sizeof(p));
    LittleFS.remove(p);
    return ESP_OK;
}

esp_err_t nvs_set_i16(nvs_handle_t h, const char* key, int16_t v)
{ return nvs_set_blob(h, key, &v, sizeof(v)); }

esp_err_t nvs_get_i16(nvs_handle_t h, const char* key, int16_t* v)
{ size_t n = sizeof(*v); return nvs_get_blob(h, key, v, &n); }

esp_err_t nvs_set_u32(nvs_handle_t h, const char* key, uint32_t v)
{ return nvs_set_blob(h, key, &v, sizeof(v)); }

esp_err_t nvs_get_u32(nvs_handle_t h, const char* key, uint32_t* v)
{ size_t n = sizeof(*v); return nvs_get_blob(h, key, v, &n); }

esp_err_t nvs_commit(nvs_handle_t) { return ESP_OK; }   // write() is the commit
void      nvs_close(nvs_handle_t)  {}
esp_err_t nvs_flash_init(void)     { return LittleFS.begin() ? ESP_OK : -1; }

int64_t esp_timer_get_time(void)
{
    return (int64_t)to_us_since_boot(get_absolute_time());
}

esp_reset_reason_t esp_reset_reason(void)
{
    return watchdog_caused_reboot() ? ESP_RST_WDT : ESP_RST_POWERON;
}

}
#endif // ARDUINO_ARCH_RP2040
