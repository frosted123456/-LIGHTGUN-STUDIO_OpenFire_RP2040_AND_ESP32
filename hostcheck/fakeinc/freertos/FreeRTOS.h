#pragma once
#include <stdint.h>
typedef void* TaskHandle_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define portMAX_DELAY 0xFFFFFFFFu
#define pdTRUE 1
#define pdPASS 1
#define tskNO_AFFINITY 0x7FFFFFFF
