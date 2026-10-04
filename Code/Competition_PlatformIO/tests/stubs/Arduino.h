#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <deque>
#include <algorithm>
#include <cstring>
#define ASHNAZG_DRY_RUN_ONLY 1
#define OUTPUT 1
#define LOW 0
#define HIGH 1
#define APB_CLK_FREQ 80000000UL
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
inline void pinMode(int,int) {}
inline void digitalWrite(int pin,int level) {if(level!=LOW && pin!=8)abort();}
inline uint64_t mockTime=0;
inline void delayMicroseconds(unsigned us) {mockTime+=us;}
inline void delay(unsigned ms){mockTime+=ms*1000;}
inline uint32_t millis(){return uint32_t(mockTime/1000);}
inline uint64_t esp_timer_get_time(){return mockTime;}
using std::min;using std::max;
constexpr int SERIAL_8N1=0;
struct FakeUart {
    std::deque<uint8_t> bytes;bool initialized=false;
    void setRxBufferSize(unsigned){}
    void begin(unsigned baud,int,int rx,int tx){if(baud!=416666 || rx!=16 || tx!=5)abort();initialized=true;}
    void end(){initialized=false;}
    explicit operator bool()const{return initialized;}
    int available()const{return int(bytes.size());}
    int read(){auto b=bytes.front();bytes.pop_front();return b;}
};
inline FakeUart Serial1;
// A distinct name avoids redeclaring the fortified glibc implementation.
inline size_t ashMockStrlcpy(char* dst,const char* src,size_t cap){size_t n=strlen(src);if(cap){size_t k=std::min(n,cap-1);memcpy(dst,src,k);dst[k]=0;}return n;}
#define strlcpy ashMockStrlcpy
struct FakeSerial { template<class T> void print(T){} template<class T> void println(T){} };
static FakeSerial Serial;
