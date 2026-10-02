#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Adafruit_DotStar.h>
#include <CRSFforArduino.hpp>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <lwip/sockets.h>
#include <errno.h>
#include <mbedtls/sha256.h>
#include "DShotESC.h"
#include "ControlCore.h"
#include "SensorHealth.h"
#include "RcObserver.h"
#include "RunCsv.h"

static const char BUILD_NAME[]="AshNazg competition 1.2";
static portMUX_TYPE stateMux=portMUX_INITIALIZER_UNLOCKED;
struct Output {
    ash::Mix wanted;int16_t left=0,right=0;
    float phase=0,rateRpm=0;uint32_t time=0,ticks=0,errors=0,maxGap=0,skipped=0,generation=0,stopUs=0;
    uint16_t computeUs=0;ash::Stop stop=ash::Stop::BOOT;bool armed=false,escReady=false,pinsLow=false;
};
struct Shared {
    ash::Rc rc;ash::Sensor sensor;Output output;
    ash::SensorFault sensorFault=ash::SensorFault::NONE;
    ash::SensorRegisters sensorRegisters,sensorLastBad;
    uint32_t sensorFaultUs=0,sensorBadReads=0,calibrationCount=0,calibrationUs=0;
    uint32_t heartbeat=0,trimChanged=0;float trim=1;
    bool fatal=false,updating=false,exporting=false,saving=false,settingsReady=false,trimVisible=false,runFrozen=true,trimSaved=false,trimSaveFailed=false;
};
static Shared state;
static Shared snapshot(){portENTER_CRITICAL(&stateMux);Shared s=state;portEXIT_CRITICAL(&stateMux);return s;}
static bool maintenance=true,wifiRunning=false,settingsSaveFailed=false;
static uint32_t captureStart=0,lastRowUs=0,rowsTotal=0,rowsRetained=0,writeIndex=0,exportId=0,downloadFailures=0;
static uint32_t captureGeneration=0,apFailures=0,zeroAckTimeouts=0;
static ash::CaptureWindow captureWindow;
static BaseRow baseRows[1500];
static WebServer baselineServer(80);
static Preferences preferences;
static ash::Trim trim;
static DShotESC escR,escL;
static bool rightInstalled=false,leftInstalled=false;
static TaskHandle_t motorTaskHandle=nullptr;
static TaskHandle_t networkTaskHandle=nullptr;
static esp_timer_handle_t motorTimer=nullptr;
static Adafruit_DotStar topStrip(9,9,7,DOTSTAR_BGR);
static CRSFforArduino crsf(&Serial1,16,5);
static RcObserver rcObserver;
static char otaStatus[64]="Ready";
static esp_reset_reason_t resetReason;
static void feedBaselineWatchdog(){if(esp_task_wdt_status(nullptr)==ESP_OK)esp_task_wdt_reset();}

static void onRaw(int8_t byte) {
    uint32_t now=uint32_t(esp_timer_get_time());bool controls=rcObserver.feed(uint8_t(byte),now);
    ash::Rc r=snapshot().rc;
    if(controls){for(unsigned i=0;i<8;++i)r.ch[i]=ash::channelPercent(rcObserver.channels[i]);r.time=rcObserver.lastRcUs;r.seen=true;}
    r.frames=rcObserver.frames;r.crcErrors=rcObserver.crcErrors;
    r.linkTime=rcObserver.lastLinkUs;r.linkSeen=rcObserver.linkFrames>0;r.lq=rcObserver.lq;
    portENTER_CRITICAL(&stateMux);state.rc=r;portEXIT_CRITICAL(&stateMux);
}
static void receiverTask(void*) {
    if(!crsf.begin()){portENTER_CRITICAL(&stateMux);state.fatal=true;portEXIT_CRITICAL(&stateMux);vTaskDelete(nullptr);}
    for(;;){crsf.update();vTaskDelay(1);}
}

static uint8_t sensorReg(uint8_t address) {
    SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));digitalWrite(8,LOW);
    SPI.transfer(address|0x80);uint8_t value=SPI.transfer(0xff);digitalWrite(8,HIGH);SPI.endTransaction();return value;
}
static void sensorWrite(uint8_t address,uint8_t value) {
    SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));digitalWrite(8,LOW);
    SPI.transfer(address&0x7f);SPI.transfer(value);digitalWrite(8,HIGH);SPI.endTransaction();
}
static void sensorXYZ(int16_t* xyz) {
    SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));digitalWrite(8,LOW);SPI.transfer(0xe8);
    for(unsigned i=0;i<3;++i){uint8_t lo=SPI.transfer(0xff),hi=SPI.transfer(0xff);xyz[i]=int16_t(lo|(uint16_t(hi)<<8))>>4;}
    digitalWrite(8,HIGH);SPI.endTransaction();
}
static void sensorTask(void*) {
    pinMode(8,OUTPUT);digitalWrite(8,HIGH);SPI.begin(36,37,35,8);
    sensorWrite(0x20,0x37);vTaskDelay(pdMS_TO_TICKS(10));sensorWrite(0x23,0xb0);vTaskDelay(pdMS_TO_TICKS(100));
    ash::Sensor s;ash::SensorRegisters registers,lastBad;uint32_t badReads=0;
    ash::SensorFault fault=ash::SensorFault::NONE;uint32_t faultUs=0;
    auto checkRegisters=[&](){return ash::verifySensorRegisters(sensorReg,[]{vTaskDelay(1);},registers,lastBad,badReads);};
    bool initialOK=checkRegisters();s.who=registers.who;s.ctrl1=registers.ctrl1;s.ctrl4=registers.ctrl4;
    uint32_t start=uint32_t(esp_timer_get_time()),checkAt=start;
    ash::SensorCalibration calibration(start);
    auto publish=[&]() {
        portENTER_CRITICAL(&stateMux);state.sensor=s;state.sensorFault=fault;state.sensorFaultUs=faultUs;
        state.sensorRegisters=registers;state.sensorLastBad=lastBad;state.sensorBadReads=badReads;
        state.calibrationCount=calibration.count;state.calibrationUs=calibration.done?calibration.elapsed:ash::age(uint32_t(esp_timer_get_time()),start);portEXIT_CRITICAL(&stateMux);
    };
    if(!initialOK){s.failed=true;fault=ash::SensorFault::STARTUP_REGISTERS;faultUs=start;publish();vTaskDelete(nullptr);}
    ash::Estimator estimator;
    for(;;) {
        uint32_t now=uint32_t(esp_timer_get_time());uint8_t status=sensorReg(0x27);++s.polls;s.status=status;
        if(status&8) {
            int16_t xyz[3];sensorXYZ(xyz);s.time=uint32_t(esp_timer_get_time());++s.fresh;
            s.x=xyz[0];s.y=xyz[1];s.z=xyz[2];if(status&128)++s.overruns;
            if(abs(s.y)>=2000)s.rail=true;
            if(!s.calibrated){calibration.sample(s.y,s.time);if(calibration.done){s.biasCounts=calibration.bias;s.calibrated=true;}}
            else {s.radialG=(s.y-s.biasCounts)*ash::SENSOR_G_PER_COUNT;estimator.sample(s.radialG,s.time);s.filteredG=estimator.filtered;s.rpm=fabsf(s.filteredG)<0.75f?0:ash::rpmFromG(s.filteredG);}
        }
        // Calibration cannot complete after a timeout and disguise a failed startup as ready.
        if(calibration.expired(uint32_t(esp_timer_get_time()))){s.failed=true;fault=ash::SensorFault::CALIBRATION_TIMEOUT;}
        // SPI owner verifies configuration periodically; a disconnected bus may otherwise look like valid 0xff status.
        if(!s.failed && ash::age(now,checkAt)>=500000){checkAt=now;if(!checkRegisters()){s.failed=true;fault=ash::SensorFault::CONFIG_REGISTERS;}}
        if(s.failed)faultUs=uint32_t(esp_timer_get_time());
        publish();if(s.failed)vTaskDelete(nullptr);vTaskDelay(1);
    }
}

static bool startupInterrupted(){Shared s=snapshot();return s.updating || s.fatal;}
#include "EscStartup.inc"
static void motorWake(void*){if(motorTaskHandle)xTaskNotifyGive(motorTaskHandle);}
static void releaseEscs() {
    if(rightInstalled){escR.WaitTXDone(pdMS_TO_TICKS(20));escR.uninstall();rightInstalled=false;}
    if(leftInstalled){escL.WaitTXDone(pdMS_TO_TICKS(20));escL.uninstall();leftInstalled=false;}
    pinMode(17,OUTPUT);digitalWrite(17,LOW);pinMode(18,OUTPUT);digitalWrite(18,LOW);
    portENTER_CRITICAL(&stateMux);state.fatal=true;state.output.armed=false;state.output.left=state.output.right=0;
    state.output.escReady=false;state.output.pinsLow=true;state.output.stop=ash::Stop::OUTPUT_ERROR;++state.output.errors;portEXIT_CRITICAL(&stateMux);
}
static void motorTask(void*) {
    if(!startupEscs()){releaseEscs();vTaskDelete(nullptr);}
    esp_timer_create_args_t args{};args.callback=motorWake;args.dispatch_method=ESP_TIMER_TASK;args.name="motor_wake";args.skip_unhandled_events=true;
    if(esp_timer_create(&args,&motorTimer)!=ESP_OK || esp_timer_start_periodic(motorTimer,166)!=ESP_OK){releaseEscs();vTaskDelete(nullptr);}
    portENTER_CRITICAL(&stateMux);state.output.escReady=true;portEXIT_CRITICAL(&stateMux);
    ash::ArmGate gate;ash::Phase phase;Output out;out.escReady=true;bool spinDetected=false;
    uint32_t previousOutput=0,previousMaintenance=0,previousGeneration=0;
    for(;;) {
        uint32_t notifications=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        uint32_t now=uint32_t(esp_timer_get_time());Shared s=snapshot();
        ash::SafetyInputs i;i.rc=s.rc;i.sensor=s.sensor;i.heartbeat=s.heartbeat;
        i.ready=s.output.escReady&&s.settingsReady;i.fatal=s.fatal;i.updating=s.updating;i.exporting=s.exporting;i.saving=s.saving;
        bool permit=gate.step(i,now);
        if(gate.generation!=previousGeneration){previousGeneration=gate.generation;phase.reset(now);spinDetected=false;out.maxGap=0;out.skipped=0;previousOutput=0;}
        bool reverse=s.rc.ch[5]>50;
        if(!spinDetected && s.sensor.rpm>=450)spinDetected=true;
        if(spinDetected && s.sensor.rpm<=350)spinDetected=false;
        float heading=1.0f+(s.rc.ch[0]-50)*0.001f*(reverse?-1.0f:1.0f);
        out.rateRpm=spinDetected?s.sensor.rpm/(s.trim*heading):0;
        phase.step(now,out.rateRpm);out.phase=phase.degrees;
        ash::Mix m;
        if(permit) {
            if(s.rc.ch[2]>10)m=ash::melty(s.rc.ch[2],spinDetected?s.rc.ch[1]:50,reverse,s.sensor.rpm,phase.degrees);
            else m=ash::ground(s.rc);
        }
        // Do not burst catch-up frames. Maintenance needs only repeated stops.
        if(!permit && previousMaintenance && ash::age(now,previousMaintenance)<4000){taskYIELD();continue;}
        if(previousOutput && ash::age(now,previousOutput)<100){++out.skipped;continue;}
        if(!permit)previousMaintenance=now;else previousMaintenance=0;
        if(notifications>1)out.skipped+=notifications-1;
        Shared finalState=snapshot();
        if(finalState.updating || finalState.exporting || finalState.saving || finalState.fatal){gate.disarm(finalState.updating?ash::Stop::OTA:ash::Stop::INIT_FAILED,now);m={};}
        // The same owner submits both outputs; no other task touches RMT.
        esp_err_t er=escR.sendThrottle3D(m.right);delayMicroseconds(25);esp_err_t el=escL.sendThrottle3D(m.left);
        if(er!=ESP_OK || el!=ESP_OK){gate.disarm(ash::Stop::OUTPUT_ERROR,now);releaseEscs();esp_timer_stop(motorTimer);vTaskDelete(nullptr);}
        uint32_t done=uint32_t(esp_timer_get_time());
        out.maxGap=max(out.maxGap,previousOutput?ash::age(done,previousOutput):0u);previousOutput=done;
        phase.step(done,out.rateRpm);out.phase=phase.degrees;
        out.computeUs=uint16_t(min(ash::age(done,now),65535u));out.time=done;++out.ticks;
        out.left=m.left;out.right=m.right;out.wanted=m;out.armed=gate.armed;out.generation=gate.generation;
        out.stop=gate.stop;out.stopUs=gate.stoppedAt;
        portENTER_CRITICAL(&stateMux);state.output=out;portEXIT_CRITICAL(&stateMux);
    }
}

static void ledTask(void*) {
    for(;;) {
        Shared s=snapshot();uint32_t now=uint32_t(esp_timer_get_time());uint32_t color=0;
        if(s.fatal || s.sensor.failed || s.sensor.rail)color=topStrip.Color(70,0,0);
        else if(!s.output.armed && s.output.stop!=ash::Stop::BOOT && s.output.stop!=ash::Stop::KILL && s.output.stop!=ash::Stop::OTA)color=topStrip.Color(70,35,0);
        else if(s.trimVisible) {
            float level=ash::clamp((s.trim-0.9f)*45.0f,0,9);
            for(unsigned p=0;p<9;++p){uint8_t b=ash::trimPixel(level,p);topStrip.setPixelColor(p,b,b,b);}
            topStrip.show();vTaskDelay(1);continue;
        }
        else if(!s.output.armed)color=topStrip.Color(0,0,s.updating?70:33);
        else if(s.rc.ch[2]<=10 || s.sensor.rpm<350)color=topStrip.Color(0,15,0);
        else {
            float phase=ash::wrap(s.output.phase+ash::age(now,s.output.time)*s.output.rateRpm*0.000006f);
            // At high RPM widen the marker to cover the one-tick LED update interval.
            float halfWidth=fmaxf(5.0f,s.output.rateRpm*0.000006f*600.0f);
            float distance=fminf(phase,360-phase);
            if(distance<=halfWidth)color=topStrip.Color(85,85,85);
            else if(s.output.wanted.mode==ash::Drive::TRANSLATE && s.output.wanted.delta!=0)color=topStrip.Color(0,20,0);
        }
        topStrip.fill(color);topStrip.show();vTaskDelay(1);
    }
}

static bool slotsOK() {
    auto a=esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_APP_OTA_0,nullptr);
    auto b=esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_APP_OTA_1,nullptr);
    return a && b && a->address==0x10000 && b->address==0x1f0000 && a->size==0x1e0000 && b->size==0x1e0000;
}
static bool zeroAcknowledged() {auto s=snapshot();return s.output.pinsLow || (s.output.escReady && !s.output.armed && s.output.left==0 && s.output.right==0 && s.output.ticks>0);}
static bool waitForZero() {
    uint32_t start=millis();while(!zeroAcknowledged() && millis()-start<100){feedBaselineWatchdog();delay(1);}
    return zeroAcknowledged();
}
static void startMaintenance() {
    if(wifiRunning)return;
    WiFi.persistent(false);WiFi.mode(WIFI_AP);bool ok=WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
    ok=WiFi.softAP("AshNazg Diagnostics","AshNazgDiag") && ok;
    if(!ok){++apFailures;return;}
    baselineServer.begin();ArduinoOTA.begin();wifiRunning=true;
}
static void stopMaintenance() {if(!wifiRunning)return;ArduinoOTA.end();baselineServer.stop();WiFi.softAPdisconnect(true);WiFi.mode(WIFI_OFF);wifiRunning=false;}
static void recordRun(bool force=false) {
    if(!captureStart)return;uint32_t now=uint32_t(esp_timer_get_time());Shared s=snapshot();
    if(!captureWindow.keep(s.output.wanted.mode,now,force))return;
    if(!force && rowsTotal && ash::age(now,lastRowUs)<10000)return;
    lastRowUs=now;BaseRow r{};
    r.us=now-captureStart;r.sensorUs=s.sensor.time-captureStart;r.polls=s.sensor.polls;r.fresh=s.sensor.fresh;r.overruns=s.sensor.overruns;
    r.rcAge=ash::age(now,s.rc.time);r.outputGap=s.output.maxGap;r.errors=s.output.errors;r.outputTicks=s.output.ticks;r.freshAge=ash::age(now,s.sensor.time);
    r.x=s.sensor.x;r.y=s.sensor.y;r.z=s.sensor.z;r.ch3=s.rc.ch[2];r.translation=s.rc.ch[1];r.ch5=s.rc.ch[4];
    r.wantedL=s.output.wanted.left;r.wantedR=s.output.wanted.right;r.left=s.output.left;r.right=s.output.right;
    r.phase=s.output.phase;r.bodyRpm=s.sensor.rpm;r.phaseRate=s.output.rateRpm;r.legacyG=s.sensor.filteredG;r.trim=s.trim;
    r.status=s.sensor.status;r.stop=unsigned(s.output.stop);r.mode=unsigned(s.output.wanted.mode);r.lq=s.rc.lq;
    r.base=s.output.wanted.base;r.delta=s.output.wanted.delta;r.computeUs=s.output.computeUs;
    baseRows[writeIndex]=r;writeIndex=(writeIndex+1)%1500;if(rowsRetained<1500)++rowsRetained;++rowsTotal;
}
static String baselineSummary() {
    Shared s=snapshot();String out=String(BUILD_NAME)+"\n";
    out+="maintenance="+String(maintenance)+"\narmed="+String(s.output.armed)+"\nmotors_inhibited="+String(!s.output.armed)+"\n";
    out+="hardware_ready="+String(s.output.escReady&&s.sensor.calibrated&&s.settingsReady&&!s.fatal&&!s.sensor.failed&&!s.sensor.rail)+"\nfatal="+String(s.fatal)+"\nsensor_failed="+String(s.sensor.failed)+"\n";
    out+="sensor_fault="+String(ash::sensorFaultName(s.sensorFault))+"\nsensor_fault_boot_us="+String(s.sensorFaultUs)+"\nsensor_calibrated="+String(s.sensor.calibrated)+"\nsensor_saturated="+String(s.sensor.rail)+"\n";
    out+="calibration_samples="+String(s.calibrationCount)+"\ncalibration_elapsed_us="+String(s.calibrationUs)+"\nsensor_bad_register_reads="+String(s.sensorBadReads)+"\n";
    out+="latest_who=0x"+String(s.sensorRegisters.who,HEX)+"\nlatest_ctrl1=0x"+String(s.sensorRegisters.ctrl1,HEX)+"\nlatest_ctrl4=0x"+String(s.sensorRegisters.ctrl4,HEX)+"\n";
    out+="last_bad_who=0x"+String(s.sensorLastBad.who,HEX)+"\nlast_bad_ctrl1=0x"+String(s.sensorLastBad.ctrl1,HEX)+"\nlast_bad_ctrl4=0x"+String(s.sensorLastBad.ctrl4,HEX)+"\n";
    out+="stop_reason="+String(ash::stopName(s.output.stop))+"\nstop_code="+String(unsigned(s.output.stop))+"\n";
    out+="stop_time_from_capture_us="+String(captureStart && s.output.stop!=ash::Stop::BOOT?uint32_t(s.output.stopUs-captureStart):0)+"\n";
    out+="rows_retained="+String(rowsRetained)+"\nrows_total="+String(rowsTotal)+"\narm_generation="+String(s.output.generation)+"\n";
    out+="capture_idle_tail_us=1000000\ncapture_idle_preserves_driving_rows=true\ntranslation_waveform=full_cosine\n";
    out+="who_am_i=0x"+String(s.sensor.who,HEX)+"\nctrl1=0x"+String(s.sensor.ctrl1,HEX)+"\nctrl4=0x"+String(s.sensor.ctrl4,HEX)+"\n";
    out+="sensor_scale_g=400\nsensor_odr_hz=400\nsensor_g_per_count=0.195\nsensor_radius_mm=18\n";
    out+="bias_counts="+String(s.sensor.biasCounts,6)+"\nphysical_rpm="+String(s.sensor.rpm,3)+"\ntrim_period="+String(s.trim,6)+"\ntrim_led_level="+String(ash::clamp((s.trim-0.9f)*45.0f,0,9),3)+"\n";
    out+="trim_saved="+String(s.trimSaved)+"\ntrim_save_failed="+String(s.trimSaveFailed)+"\n";
    out+="validated_RC_frames="+String(s.rc.frames)+"\nRC_crc_errors="+String(s.rc.crcErrors)+"\nlink_quality="+String(s.rc.lq)+"\nlink_statistics_seen="+String(s.rc.linkSeen)+"\n";
    out+="sensor_ready_count="+String(s.sensor.fresh)+"\nsensor_overruns="+String(s.sensor.overruns)+"\noutput_iterations="+String(s.output.ticks)+"\n";
    out+="output_errors="+String(s.output.errors)+"\nmax_output_gap_us="+String(s.output.maxGap)+"\nnotification_skips="+String(s.output.skipped)+"\n";
    out+="ota_status="+String(otaStatus)+"\nOTA_slots_valid="+String(slotsOK())+"\nreset_reason_code="+String(unsigned(resetReason))+"\n";
    auto running=esp_ota_get_running_partition();auto next=esp_ota_get_next_update_partition(nullptr);
    out+="running_slot="+String(running?running->label:"unavailable")+"\nnext_slot="+String(next?next->label:"unavailable")+"\nnext_slot_bytes="+String(next?next->size:0)+"\n";
    out+="free_heap="+String(ESP.getFreeHeap())+"\ndownload_failures="+String(downloadFailures)+"\nAP_start_failures="+String(apFailures)+"\nzero_ack_timeouts="+String(zeroAckTimeouts)+"\n";
    out+="driver_acceptance_is_not_ESC_acknowledgement=true\nno_ESC_settings_saved=true\n";return out;
}
#include "RunExport.inc"

static bool beginExport() {
    portENTER_CRITICAL(&stateMux);bool ok=!state.output.armed && state.runFrozen && !state.updating && !state.saving;
    if(ok)state.exporting=true;portEXIT_CRITICAL(&stateMux);
    if(!ok){baselineServer.send(409,"text/plain","Stop with CH5 HIGH before downloading.");return false;}
    if(!waitForZero()){portENTER_CRITICAL(&stateMux);state.exporting=false;portEXIT_CRITICAL(&stateMux);baselineServer.send(503,"text/plain","Waiting for inhibited motor owner.");return false;}
    return true;
}
static void endExport(){portENTER_CRITICAL(&stateMux);state.exporting=false;portEXIT_CRITICAL(&stateMux);}
static void saveTrim() {
    static uint32_t lastAttempt=0;static bool attempted=false;
    if(!trim.dirty || snapshot().output.armed || !zeroAcknowledged() || snapshot().updating)return;
    if(attempted && uint32_t(millis()-lastAttempt)<1000)return;
    portENTER_CRITICAL(&stateMux);bool allowed=!state.output.armed && !state.exporting && !state.updating;
    if(allowed)state.saving=true;portEXIT_CRITICAL(&stateMux);if(!allowed)return;
    attempted=true;lastAttempt=millis();
    if(waitForZero()) {
        auto record=ash::trimRecord(trim.period);
        bool ok=preferences.begin("ashnazg",false);
        if(ok){ok=preferences.putBytes("trim-v1",&record,sizeof(record))==sizeof(record);preferences.end();}
        settingsSaveFailed=!ok;if(ok)trim.dirty=false;
    }
    portENTER_CRITICAL(&stateMux);state.saving=false;state.settingsReady=!settingsSaveFailed;portEXIT_CRITICAL(&stateMux);
}
static void networkTask(void*) {
    for(;;) {
        Shared s=snapshot();
        if(s.output.armed)stopMaintenance();
        else {
            startMaintenance();
            if(s.runFrozen){ArduinoOTA.handle();baselineServer.handleClient();}
        }
        vTaskDelay(1);
    }
}
void setup() {
    pinMode(17,OUTPUT);digitalWrite(17,LOW);pinMode(18,OUTPUT);digitalWrite(18,LOW);
    resetReason=esp_reset_reason();exportId=esp_random();Serial.begin(115200);topStrip.begin();topStrip.setBrightness(255);topStrip.fill(topStrip.Color(0,0,33));topStrip.show();
    ArduinoOTA.setHostname("ashnazg-competition");ArduinoOTA.setPassword("admin");
    ArduinoOTA.onStart([] {
        portENTER_CRITICAL(&stateMux);state.updating=true;portEXIT_CRITICAL(&stateMux);strlcpy(otaStatus,"Uploading; motor output locked",sizeof(otaStatus));
        if(!waitForZero()){++zeroAckTimeouts;ESP.restart();}
    });
    ArduinoOTA.onError([](ota_error_t error){snprintf(otaStatus,sizeof(otaStatus),"Upload error %u; output locked",unsigned(error));});
    baselineServer.on("/",[] {
        if(!beginExport())return;
        String p=String("<h1>")+BUILD_NAME+"</h1><p>CH5 HIGH stops and enables Wi-Fi. To arm: hardware ready, throttle zero, CH1/CH2 centered, hold CH5 HIGH for one second, then LOW.</p>";
        p+="<p>Keep still during startup calibration. No automatic spin or test duration limit. Download before rearming; logs stay in RAM.</p><p><a href='/summary.txt'>Summary</a> | <a href='/run.csv'>CSV</a></p><pre>"+baselineSummary()+"</pre>";baselineServer.send(200,"text/html",p);endExport();
    });
    baselineServer.on("/status",[]{if(beginExport()){baselineServer.send(200,"text/plain",baselineSummary());endExport();}});
    baselineServer.on("/summary.txt",[]{if(beginExport()){baselineServer.sendHeader("Content-Disposition","attachment; filename=AshNazg-competition-summary.txt");baselineServer.send(200,"text/plain",baselineSummary());endExport();}});
    baselineServer.on("/run.csv",[]{if(beginExport()){downloadBaseline();endExport();}});
    baselineServer.on("/download-check.csv",[]{if(beginExport()){downloadExport(true);endExport();}});
    baselineServer.on("/export.json",[]{if(beginExport()){exportManifest();endExport();}});
    const char* headers[]={"Range","If-Range"};baselineServer.collectHeaders(headers,2);
    startMaintenance();
    // Versioned record replaces legacy EEPROM trim without erasing rollback settings.
    ash::TrimRecord record{};bool valid=false;
    if(preferences.begin("ashnazg",true)){if(preferences.getBytesLength("trim-v1")==sizeof(record))valid=preferences.getBytes("trim-v1",&record,sizeof(record))==sizeof(record)&&ash::trimValid(record);preferences.end();}
    if(valid)trim.period=record.period;else trim.dirty=true;
    bool slotValid=slotsOK();
    portENTER_CRITICAL(&stateMux);state.trim=trim.period;state.settingsReady=valid;state.heartbeat=uint32_t(esp_timer_get_time());state.fatal=!slotValid;portEXIT_CRITICAL(&stateMux);
    crsf.setRawDataCallback(onRaw);
    if(xTaskCreatePinnedToCore(receiverTask,"receiver",4096,nullptr,1,nullptr,0)!=pdPASS ||
       xTaskCreatePinnedToCore(sensorTask,"sensor",4096,nullptr,1,nullptr,0)!=pdPASS ||
       xTaskCreatePinnedToCore(ledTask,"led",4096,nullptr,1,nullptr,0)!=pdPASS ||
       xTaskCreatePinnedToCore(motorTask,"motor",6144,nullptr,2,&motorTaskHandle,1)!=pdPASS ||
       xTaskCreatePinnedToCore(networkTask,"network",8192,nullptr,1,&networkTaskHandle,0)!=pdPASS) {
        portENTER_CRITICAL(&stateMux);state.fatal=true;state.output.pinsLow=!motorTaskHandle;portEXIT_CRITICAL(&stateMux);
    }
    esp_task_wdt_add(nullptr);
}
void loop() {
    feedBaselineWatchdog();uint32_t now=uint32_t(esp_timer_get_time());
    portENTER_CRITICAL(&stateMux);state.heartbeat=now;portEXIT_CRITICAL(&stateMux);
    Shared s=snapshot();
    if(ash::rcFresh(s.rc,now) && ash::linkHealthy(s.rc,now) && !s.updating)trim.adjust(s.rc.ch[3],now);
    portENTER_CRITICAL(&stateMux);state.trim=trim.period;state.trimVisible=trim.visible(now);state.trimChanged=trim.changedAt;
    state.trimSaved=!trim.dirty;state.trimSaveFailed=settingsSaveFailed;portEXIT_CRITICAL(&stateMux);
    if(s.output.armed) {
        if(maintenance){maintenance=false;portENTER_CRITICAL(&stateMux);state.runFrozen=false;portEXIT_CRITICAL(&stateMux);}
        if(s.output.generation!=captureGeneration){captureGeneration=s.output.generation;captureStart=0;rowsTotal=rowsRetained=writeIndex=0;captureWindow={};exportId=esp_random();}
        if(!captureStart && s.output.wanted.mode!=ash::Drive::STOP)captureStart=now;
        recordRun();
    } else {
        if(!maintenance){recordRun(true);maintenance=true;portENTER_CRITICAL(&stateMux);state.runFrozen=true;portEXIT_CRITICAL(&stateMux);}
        saveTrim();
        // A worker allocation failure must not remove the software update route.
        if(!networkTaskHandle){startMaintenance();ArduinoOTA.handle();baselineServer.handleClient();}
    }
    delay(1);
}
