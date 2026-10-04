#pragma once
#include <stdint.h>
using TickType_t=uint32_t;
#define portTICK_PERIOD_MS 1
inline void portYIELD(){}
inline void vTaskDelay(TickType_t ticks){delay(ticks);}
inline TickType_t xTaskGetTickCount(){static TickType_t t=0;return ++t;}
using TaskHandle_t=void*;
inline void xTaskNotifyGive(TaskHandle_t){}
inline uint32_t ulTaskNotifyTake(bool,TickType_t ticks){delay(ticks);return 0;}
#define pdTRUE true
