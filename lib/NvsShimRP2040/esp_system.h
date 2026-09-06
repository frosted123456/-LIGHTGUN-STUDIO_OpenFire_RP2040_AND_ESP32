// esp_system shim for RP2040: the reset reason from the chip's own registers.
#pragma once
typedef enum {
    ESP_RST_UNKNOWN = 0, ESP_RST_POWERON, ESP_RST_EXT, ESP_RST_SW,
    ESP_RST_PANIC, ESP_RST_INT_WDT, ESP_RST_TASK_WDT, ESP_RST_WDT,
    ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT, ESP_RST_SDIO,
} esp_reset_reason_t;
#ifdef __cplusplus
extern "C" {
#endif
esp_reset_reason_t esp_reset_reason(void);
// The RP2040 reason by name, for '~ping': POR (power-on/brown-out), RUN (reset
// pin), WDT_FORCE (a forced reboot: UF2 upload, sClearFlash), WDT (timer), DBG.
const char* rp2040_reset_reason(void);
#ifdef __cplusplus
}
#endif
