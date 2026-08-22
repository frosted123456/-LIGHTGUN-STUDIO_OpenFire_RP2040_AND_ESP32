// Host stand-in for esp_system.h: just enough for aim_runtime's boot forensics.
#pragma once
typedef enum {
    ESP_RST_UNKNOWN = 0, ESP_RST_POWERON, ESP_RST_EXT, ESP_RST_SW,
    ESP_RST_PANIC, ESP_RST_INT_WDT, ESP_RST_TASK_WDT, ESP_RST_WDT,
    ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT, ESP_RST_SDIO,
} esp_reset_reason_t;
static inline esp_reset_reason_t esp_reset_reason(void) { return ESP_RST_POWERON; }
