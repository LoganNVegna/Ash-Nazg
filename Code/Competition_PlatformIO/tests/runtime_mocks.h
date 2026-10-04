#pragma once
#include "Arduino.h"
#include "freertos/task.h"
#include <functional>
#include <vector>
#include <assert.h>
constexpr int GPIO_NUM_18=18,GPIO_NUM_17=17,SPI_MODE0=0,MSBFIRST=0,ESP_TIMER_TASK=0;
struct SPISettings{SPISettings(unsigned,int,int){}};
struct MockSPI {
    uint8_t registers[64]{};bool first=true,reading=false;unsigned address=0;
    MockSPI(){registers[0x0f]=0x32;registers[0x27]=8;xyz(0,-16,0);}
    void xyz(int x,int y,int z){int values[]={x,y,z};for(unsigned i=0;i<3;++i){uint16_t v=uint16_t(values[i]*16);registers[0x28+2*i]=uint8_t(v);registers[0x29+2*i]=uint8_t(v>>8);}}
    void begin(int,int,int,int){}
    void beginTransaction(SPISettings){first=true;}
    uint8_t transfer(uint8_t v){if(first){first=false;reading=v&0x80;address=v&0x3f;return 0;}if(reading)return registers[address++%64];registers[address++%64]=v;return 0;}
    void endTransaction(){}
};
inline MockSPI SPI;
struct MockStrip {
    uint32_t pixels[9]{};unsigned shows=0;
    uint32_t Color(uint8_t r,uint8_t g,uint8_t b){return (uint32_t(r)<<16)|(uint32_t(g)<<8)|b;}
    void clear(){for(auto& p:pixels)p=0;}
    void fill(uint32_t color){for(auto& p:pixels)p=color;}
    void setPixelColor(unsigned i,uint32_t c){assert(i<9);pixels[i]=c;}
    void setPixelColor(unsigned i,uint8_t r,uint8_t g,uint8_t b){setPixelColor(i,Color(r,g,b));}
    void show(){++shows;}
};
inline MockStrip topStrip;
using esp_timer_handle_t=void*;
struct esp_timer_create_args_t{void(*callback)(void*);int dispatch_method;const char* name;bool skip_unhandled_events;};
inline int esp_timer_create(esp_timer_create_args_t*,esp_timer_handle_t*){return 1;}
inline int esp_timer_start_periodic(esp_timer_handle_t,unsigned){return 1;}
inline void esp_timer_delete(esp_timer_handle_t){}
using ota_error_t=int;
struct MockOTA {
    std::function<void()> start;std::function<void(int)> error;std::function<void(unsigned,unsigned)> progress;
    std::function<void()> handler;unsigned handles=0;
    void onStart(std::function<void()> f){start=f;}
    void onError(std::function<void(int)> f){error=f;}
    void onProgress(std::function<void(unsigned,unsigned)> f){progress=f;}
    void handle(){++handles;if(handler)handler();}
};
inline MockOTA ArduinoOTA;
struct MockUpdate{unsigned aborts=0;void abort(){++aborts;}};
inline MockUpdate Update;
struct MockServer{unsigned handles=0;void handleClient(){++handles;}};
inline MockServer baselineServer;
struct MockPreferences {
    bool available=false;unsigned writes=0;
    bool begin(const char*,bool){return available;}
    size_t putBytes(const char*,const void*,size_t n){++writes;return available?n:0;}
    size_t putFloat(const char*,float){++writes;return available?4:0;}
    void end(){}
};
inline MockPreferences preferences;
inline bool settingsSaveFailed=false,wifiRunning=true,mockSlots=true;
inline unsigned zeroAckTimeouts=0,taskRetries=0,maintenanceStarts=0,maintenanceStops=0;
inline ash::Trim trim;
inline char otaStatus[64]="Ready";
inline void feedBaselineWatchdog(){}
inline bool slotsOK(){return mockSlots;}
inline void startMaintenance(){++maintenanceStarts;wifiRunning=true;}
inline void stopMaintenance(){++maintenanceStops;wifiRunning=false;}
constexpr int pdPASS=1;
inline bool mockAllocationFail=true;
inline unsigned allocationAttempts=0;
inline int xTaskCreatePinnedToCore(void(*)(void*),const char*,unsigned,void*,unsigned,TaskHandle_t* h,unsigned){++allocationAttempts;if(mockAllocationFail)return 0;*h=reinterpret_cast<void*>(uintptr_t(allocationAttempts));return pdPASS;}
