// esp_timer shim for RP2040: 64-bit microseconds since boot.
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int64_t esp_timer_get_time(void);
#ifdef __cplusplus
}
#endif
