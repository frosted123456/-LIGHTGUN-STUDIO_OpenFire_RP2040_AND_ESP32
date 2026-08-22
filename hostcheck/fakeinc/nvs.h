// Minimal NVS stub for HOST LINK TESTS of the ESP_PLATFORM code path.
// Linkage matters as much as the signatures: these are extern "C" exactly like
// ESP-IDF's, so a call emitted with C++ mangling fails to link here too. That
// is the point -- see hostcheck/esp_link_test.cpp.
#pragma once
#include <stdint.h>
#include <stddef.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NVS_NOT_FOUND 0x1102

typedef uint32_t nvs_handle_t;
typedef enum { NVS_READONLY = 0, NVS_READWRITE = 1 } nvs_open_mode_t;

extern "C" {
esp_err_t nvs_open(const char* name, nvs_open_mode_t open_mode, nvs_handle_t* out_handle);
esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length);
esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length);
esp_err_t nvs_erase_key(nvs_handle_t handle, const char* key);
esp_err_t nvs_commit(nvs_handle_t handle);
esp_err_t nvs_set_i16(nvs_handle_t handle, const char* key, int16_t value);
esp_err_t nvs_get_i16(nvs_handle_t handle, const char* key, int16_t* out_value);
esp_err_t nvs_set_u32(nvs_handle_t handle, const char* key, uint32_t value);
esp_err_t nvs_get_u32(nvs_handle_t handle, const char* key, uint32_t* out_value);
void      nvs_close(nvs_handle_t handle);
}
