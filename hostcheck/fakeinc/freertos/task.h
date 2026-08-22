#pragma once
#include "FreeRTOS.h"
typedef uint8_t StackType_t;
extern "C" {
BaseType_t xTaskCreatePinnedToCore(void(*)(void*),const char*,uint32_t,void*,UBaseType_t,TaskHandle_t*,BaseType_t);
void vTaskDelay(TickType_t);
void vTaskDelete(TaskHandle_t);
UBaseType_t uxTaskPriorityGet(TaskHandle_t);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t);
}
