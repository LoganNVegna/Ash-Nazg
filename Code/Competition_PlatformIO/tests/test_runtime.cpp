#include "RecoveryCore.h"
#include "SensorHealth.h"
#include "RcObserver.h"
#include "DShotESC.h"
#include "runtime_mocks.h"
#include <iostream>
std::vector<MockFrame> trace;
static portMUX_TYPE stateMux=0;
#include "generated/runtime_state.inc"
static DShotESC escR,escL;
static TaskHandle_t motorTaskHandle=nullptr,receiverTaskHandle=nullptr,sensorTaskHandle=nullptr,networkTaskHandle=nullptr,ledTaskHandle=nullptr;
static esp_timer_handle_t motorTimer=nullptr;
static RcObserver rcObserver;
static ash::Events<> events;
#include "../src/Runtime.inc"
#include "generated/runtime_maintenance.inc"
static void sendControls(int throttle=0,bool high=true) {
    unsigned channels[16];for(auto& c:channels)c=992;
    channels[2]=throttle?591:172;channels[4]=high?1811:172;channels[5]=172;
    if(throttle)channels[1]=1811;
    uint8_t f[26]{};f[0]=0xc8;f[1]=24;f[2]=0x16;
    for(unsigned ch=0;ch<16;++ch)for(unsigned b=0;b<11;++b)f[3+(ch*11+b)/8]|=((channels[ch]>>b)&1)<<((ch*11+b)%8);
    f[25]=RcObserver::crc(f+2,23);for(auto c:f)Serial1.bytes.push_back(c);receiverTick();
}
static void stats(unsigned lq) {
    uint8_t f[14]{};f[0]=0xc8;f[1]=12;f[2]=0x14;f[5]=uint8_t(lq);f[13]=RcObserver::crc(f+2,11);
    for(auto c:f)Serial1.bytes.push_back(c);receiverTick();
}
static void advance(unsigned count,bool controls=true,bool high=true,int throttle=0) {
    for(unsigned n=0;n<count;++n){mockTime+=1000;if(controls && n%10==0)sendControls(throttle,high);sensorTick();motorTick();}
}
int main(int argc,char**) {
    bool offline=argc>1;
    if(offline)SPI.registers[0x0f]=0xff;
    mockTime=1000;ensureTasks();assert(allocationAttempts==5 && taskRetries==5);
    // Execute the cooperative fallbacks while every task allocation fails.
    advance(3000);assert(state.output.leftReady && state.output.rightReady && !state.output.armed);
    if(!offline)assert(state.sensor.calibrated && fabsf(state.sensor.biasCounts+16)<.001f);
    installCallbacks();networkTick();assert(ArduinoOTA.handles==1 && !state.networkBusy);
    mockTime+=1000;sendControls(0,true);motorTick();mockTime+=1000;sendControls(0,false);motorTick();
    assert(state.output.armed);SPI.xyz(0,-56,0);advance(400,true,false,25);
    assert(state.output.armed && state.output.wanted.mode==ash::Drive::TRANSLATE && state.output.wanted.base==250);
    if(offline) {
        assert(state.sensor.failed && state.output.quality==ash::Quality::PREDICTED);
        SPI.registers[0x0f]=0x32;advance(1200,true,false,25);
        assert(!state.sensor.failed && state.output.armed && !state.sensor.calibrated);
        SPI.xyz(0,-16,0);sendControls(0,true);advance(4000,true,true,0);
        assert(!state.output.armed && state.sensor.calibrated);
        std::cout<<"PASS: missing sensor at boot permits deliberate drive; background register recovery and stationary calibration need no reset.\n";return 0;
    }
    auto generation=state.output.generation;
    SPI.xyz(0,-2048,0);advance(50,true,false,25);
    assert(state.sensor.rail && state.output.armed && state.output.generation==generation && state.output.wanted.base==250);
    assert(state.output.quality==ash::Quality::PREDICTED && state.output.wanted.mode==ash::Drive::TRANSLATE);
    auto phase=state.output.phase;advance(50,true,false,25);assert(state.output.phase!=phase);
    // Configuration failure retries and returns while driving; no worker exits.
    SPI.registers[0x0f]=0xff;SPI.registers[0x20]=0xff;SPI.registers[0x23]=0xff;
    advance(600,true,false,25);assert(state.sensor.failed && state.output.armed && state.output.wanted.base==250);
    SPI.registers[0x0f]=0x32;SPI.registers[0x20]=0x37;SPI.registers[0x23]=0xb0;SPI.xyz(0,-56,0);
    advance(600,true,false,25);assert(!state.sensor.failed && !state.sensor.rail && state.output.armed);
    // Single busy response doesn't replace either command with zero.
    auto rightWrites=mockWrites[3];mockFault[2]=ESP_ERR_TIMEOUT;advance(1,true,false,25);
    assert(state.output.armed && state.output.leftReady && mockWrites[3]>rightWrites);
    mockFault[2]=ESP_OK;advance(2,true,false,25);assert(state.output.leftReady);
    mockFault[2]=ESP_ERR_INVALID_STATE;rightWrites=mockWrites[3];advance(60,true,false,25);
    assert(state.output.armed && state.output.rightReady && mockWrites[3]>rightWrites && state.output.leftRecoveries>=1);
    mockFault[2]=ESP_OK;advance(150,true,false,25);assert(state.output.leftReady && state.output.armed);
    // Optional settings/UI flags cannot disarm during driving.
    state.saving=state.exporting=state.networkBusy=true;advance(10,true,false,25);assert(state.output.armed);
    state.saving=state.exporting=state.networkBusy=false;
    stats(0);uint32_t lastLive=state.rc.liveTime;
    // Simulate receiver-generated held frames after explicit RF loss.
    while(ash::age(uint32_t(mockTime),lastLive)<4900000){advance(10,true,false,25);assert(state.output.armed);}
    assert(state.rc.liveTime==lastLive);
    while(ash::age(uint32_t(mockTime),lastLive)<5002000)advance(1,false);
    assert(!state.output.armed && state.output.stop==ash::Stop::RC_STALE);
    networkTick();assert(wifiRunning);
    stats(100);sendControls(25,false);mockTime+=1000;motorTick();assert(!state.output.armed);
    sendControls(25,true);mockTime+=1000;motorTick();assert(!state.output.armed);
    sendControls(25,false);mockTime+=1000;motorTick();assert(state.output.armed);
    // Normal stop and NVS failure: RAM configuration remains usable.
    sendControls(0,true);mockTime+=1000;motorTick();advance(2,true,true,0);assert(!state.output.armed);
    trim.dirty=true;state.tuningDirty=true;saveSettings();assert(settingsSaveFailed && !state.saving);
    sendControls(0,true);mockTime+=1000;motorTick();sendControls(0,false);mockTime+=1000;motorTick();assert(state.output.armed);
    sendControls(0,true);mockTime+=1000;motorTick();advance(2,true,true,0);
    // Run actual OTA callbacks under the network's exclusive maintenance claim.
    ArduinoOTA.handler=[] {
        assert(state.networkBusy);ArduinoOTA.start();assert(state.updating);
        sendControls(0,false);mockTime+=1000;motorTick();assert(!state.output.armed);
        ArduinoOTA.error(3);assert(!state.updating);
    };
    networkTick();assert(!state.updating && !state.networkBusy && Update.aborts==1);
    ArduinoOTA.handler=nullptr;networkTick();assert(wifiRunning);
    sendControls(0,false);mockTime+=1000;motorTick();assert(!state.output.armed);
    sendControls(0,true);mockTime+=1000;motorTick();sendControls(0,false);mockTime+=1000;motorTick();assert(state.output.armed);
    // Distinct diagnostic indicators survive a sensor failure; no all-red lock.
    state.tuning.diagnosticLeds=true;state.sensor.failed=true;mockTime+=1000;sendControls(25,false);motorTick();ledTick();
    assert(topStrip.pixels[4]==topStrip.Color(18,8,0));
    mockAllocationFail=false;mockTime+=1000001;ensureTasks();assert(motorTaskHandle&&receiverTaskHandle&&networkTaskHandle&&sensorTaskHandle&&ledTaskHandle);
    std::cout<<"PASS: actual UART/SPI/output/maintenance steps: clipping, missing configuration, independent RMT errors, task-allocation fallback/retry, held-frame RF loss, CH5 recovery, NVS failure and aborted OTA.\n";
}
