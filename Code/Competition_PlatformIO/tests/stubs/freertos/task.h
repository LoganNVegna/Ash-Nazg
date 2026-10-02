#pragma once
#include <stdint.h>
using TickType_t=uint32_t;
#define portTICK_PERIOD_MS 1
inline void portYIELD(){}
inline void vTaskDelay(TickType_t){}
inline TickType_t xTaskGetTickCount(){static TickType_t t=0;return ++t;}
