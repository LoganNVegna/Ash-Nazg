#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#define ASHNAZG_DRY_RUN_ONLY 1
#define OUTPUT 1
#define LOW 0
#define APB_CLK_FREQ 80000000UL
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
inline void pinMode(int,int) {}
inline void digitalWrite(int,int level) {if(level!=LOW)abort();}
inline void delayMicroseconds(unsigned) {}
struct FakeSerial { template<class T> void print(T){} template<class T> void println(T){} };
static FakeSerial Serial;
