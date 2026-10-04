#pragma once
#include "ControlCore.h"

namespace ash {
enum class SensorFault:uint8_t {NONE,STARTUP_REGISTERS,CALIBRATION_TIMEOUT,CONFIG_REGISTERS};
inline const char* sensorFaultName(SensorFault f) {
    const char* names[]={"none","startup_register_readback","calibration_timeout","configuration_readback"};
    return names[unsigned(f)];
}
struct SensorRegisters {
    uint8_t who=0,ctrl1=0,ctrl4=0;
    bool valid()const{return who==0x32 && ctrl1==0x37 && ctrl4==0xb0;}
};
// Read retries are bounded; the SPI owner repairs configuration independently.
template<class Read,class Pause>
bool verifySensorRegisters(Read read,Pause pause,SensorRegisters& last,
                           SensorRegisters& lastBad,uint32_t& badReads) {
    for(unsigned attempt=0;attempt<3;++attempt) {
        last.who=read(0x0f);last.ctrl1=read(0x20);last.ctrl4=read(0x23);
        if(last.valid())return true;
        lastBad=last;++badReads;
        if(attempt<2)pause();
    }
    return false;
}
struct SensorCalibration {
    static constexpr uint32_t TIMEOUT_US=10000000;
    uint32_t started=0,count=0,elapsed=0;double sum=0;float bias=0;bool done=false;
    explicit SensorCalibration(uint32_t now):started(now){}
    void sample(int16_t y,uint32_t now) {
        if(done || expired(now))return;
        sum+=y;++count;elapsed=age(now,started);
        if(count==400){bias=float(sum/400);done=true;}
    }
    bool expired(uint32_t now)const{return !done && age(now,started)>=TIMEOUT_US;}
};
}
