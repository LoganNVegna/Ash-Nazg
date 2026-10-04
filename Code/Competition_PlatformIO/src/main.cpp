#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Adafruit_DotStar.h>
#include <Update.h>
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
#include "RecoveryCore.h"
#include "RcObserver.h"
#include "RunCsv.h"

static const char BUILD_NAME[]="AshNazg competition 1.4";
static portMUX_TYPE stateMux=portMUX_INITIALIZER_UNLOCKED;
struct Output {
    ash::Mix wanted;int16_t left=0,right=0;
    float phase=0,drivePhase=0,appliedPhase=0,rateRpm=0,rpm=0,learned=0;
    uint32_t time=0,ticks=0,errors=0,busy=0,maxGap=0,skipped=0,generation=0,stopUs=0;
    uint32_t leftTime=0,rightTime=0,pairTime=0,leftRecoveries=0,rightRecoveries=0,late=0;
    uint16_t computeUs=0;ash::Stop stop=ash::Stop::BOOT;ash::Quality quality=ash::Quality::UNAVAILABLE;
    bool armed=false,escReady=false,pinsLow=true,leftReady=false,rightReady=false,leftUnavailable=true,rightUnavailable=true;
};
struct Shared {
    ash::Rc rc;ash::Sensor sensor;Output output;ash::Tuning tuning;
    ash::SensorFault sensorFault=ash::SensorFault::NONE;
    ash::SensorRegisters sensorRegisters,sensorLastBad;
    uint32_t sensorFaultUs=0,sensorBadReads=0,calibrationCount=0,calibrationUs=0;
    uint32_t trimChanged=0;float trim=1,savedBias=0;
    bool updating=false,exporting=false,saving=false,networkBusy=false,trimVisible=false,trimSaved=false,trimSaveFailed=false;
    bool tuningDirty=false,biasSaved=false,runFrozen=true;
};
static Shared state;
static Shared snapshot(){portENTER_CRITICAL(&stateMux);Shared s=state;portEXIT_CRITICAL(&stateMux);return s;}
static bool maintenance=true,wifiRunning=false,settingsSaveFailed=false;
static uint32_t captureStart=0,lastRowUs=0,rowsTotal=0,rowsRetained=0,writeIndex=0,exportId=0,downloadFailures=0;
static uint32_t captureGeneration=0,apFailures=0,zeroAckTimeouts=0;
static ash::CaptureWindow captureWindow;
static constexpr unsigned BASE_ROW_CAPACITY=256;
static BaseRow baseRows[BASE_ROW_CAPACITY];
static ash::Events<> events;
static uint32_t taskRetries=0;
static WebServer baselineServer(80);
static Preferences preferences;
static ash::Trim trim;
static DShotESC escR,escL;

static TaskHandle_t motorTaskHandle=nullptr;
static TaskHandle_t networkTaskHandle=nullptr;
static TaskHandle_t receiverTaskHandle=nullptr,sensorTaskHandle=nullptr,ledTaskHandle=nullptr;
static esp_timer_handle_t motorTimer=nullptr;
static Adafruit_DotStar topStrip(9,9,7,DOTSTAR_BGR);

static RcObserver rcObserver;
static char otaStatus[64]="Ready";
static esp_reset_reason_t resetReason;
static void feedBaselineWatchdog(){if(esp_task_wdt_status(nullptr)==ESP_OK)esp_task_wdt_reset();}

#include "Runtime.inc"

static bool slotsOK() {
    auto a=esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_APP_OTA_0,nullptr);
    auto b=esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_APP_OTA_1,nullptr);
    return a && b && a->address==0x10000 && b->address==0x1f0000 && a->size==0x1e0000 && b->size==0x1e0000;
}
static bool zeroAcknowledged() {
    auto s=snapshot();auto o=s.output;
    return !o.armed && (o.leftUnavailable || (o.left==0 && int32_t(o.leftTime-o.stopUs)>=0)) &&
        (o.rightUnavailable || (o.right==0 && int32_t(o.rightTime-o.stopUs)>=0));
}
static bool waitForZero() {
    uint32_t start=millis();while(!zeroAcknowledged() && millis()-start<100){feedBaselineWatchdog();if(!motorTaskHandle)motorTick();delay(1);}
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
    if(!force && rowsTotal && ash::age(now,lastRowUs)<20000)return;
    lastRowUs=now;BaseRow r{};
    r.us=now-captureStart;r.sensorUs=s.sensor.time-captureStart;r.polls=s.sensor.polls;r.fresh=s.sensor.fresh;r.overruns=s.sensor.overruns;
    r.rcAge=ash::age(now,s.rc.time);r.outputGap=s.output.maxGap;r.errors=s.output.errors;r.outputTicks=s.output.ticks;r.freshAge=ash::age(now,s.sensor.time);
    r.x=s.sensor.x;r.y=s.sensor.y;r.z=s.sensor.z;r.ch3=s.rc.ch[2];r.translation=s.rc.ch[1];r.ch5=s.rc.ch[4];
    r.wantedL=s.output.wanted.left;r.wantedR=s.output.wanted.right;r.left=s.output.left;r.right=s.output.right;
    r.phase=s.output.phase;r.bodyRpm=s.sensor.rpm;r.phaseRate=s.output.rateRpm;r.legacyG=s.sensor.filteredG;r.trim=s.trim;
    r.status=s.sensor.status;r.stop=unsigned(s.output.stop);r.mode=unsigned(s.output.wanted.mode);r.lq=s.rc.lq;
    r.base=s.output.wanted.base;r.delta=s.output.wanted.delta;r.computeUs=s.output.computeUs;
    portENTER_CRITICAL(&stateMux);
    if(!state.runFrozen){baseRows[writeIndex]=r;writeIndex=(writeIndex+1)%BASE_ROW_CAPACITY;if(rowsRetained<BASE_ROW_CAPACITY)++rowsRetained;++rowsTotal;}
    portEXIT_CRITICAL(&stateMux);
}
static String baselineSummary() {
    Shared s=snapshot();String out=String(BUILD_NAME)+"\n";
    out+="maintenance="+String(maintenance)+"\narmed="+String(s.output.armed)+"\nmotors_inhibited="+String(!s.output.armed)+"\n";
    out+="hardware_ready="+String(s.output.escReady)+"\nfatal="+String(false)+"\nsensor_failed="+String(s.sensor.failed)+"\n";
    out+="sensor_fault="+String(ash::sensorFaultName(s.sensorFault))+"\nsensor_fault_boot_us="+String(s.sensorFaultUs)+"\nsensor_calibrated="+String(s.sensor.calibrated)+"\nsensor_saturated="+String(s.sensor.rail)+"\n";
    out+="calibration_samples="+String(s.calibrationCount)+"\ncalibration_elapsed_us="+String(s.calibrationUs)+"\nsensor_bad_register_reads="+String(s.sensorBadReads)+"\n";
    out+="latest_who=0x"+String(s.sensorRegisters.who,HEX)+"\nlatest_ctrl1=0x"+String(s.sensorRegisters.ctrl1,HEX)+"\nlatest_ctrl4=0x"+String(s.sensorRegisters.ctrl4,HEX)+"\n";
    out+="last_bad_who=0x"+String(s.sensorLastBad.who,HEX)+"\nlast_bad_ctrl1=0x"+String(s.sensorLastBad.ctrl1,HEX)+"\nlast_bad_ctrl4=0x"+String(s.sensorLastBad.ctrl4,HEX)+"\n";
    out+="stop_reason="+String(ash::stopName(s.output.stop))+"\nstop_code="+String(unsigned(s.output.stop))+"\n";
    out+="stop_time_from_capture_us="+String(captureStart && s.output.stop!=ash::Stop::BOOT?uint32_t(s.output.stopUs-captureStart):0)+"\n";
    out+="rows_retained="+String(rowsRetained)+"\nrows_total="+String(rowsTotal)+"\narm_generation="+String(s.output.generation)+"\n";
    out+="capture_idle_tail_us=1000000\ncapture_idle_preserves_driving_rows=true\n";
    out+="who_am_i=0x"+String(s.sensor.who,HEX)+"\nctrl1=0x"+String(s.sensor.ctrl1,HEX)+"\nctrl4=0x"+String(s.sensor.ctrl4,HEX)+"\n";
    out+="sensor_scale_g=400\nsensor_odr_hz=400\nsensor_g_per_count=0.195\nsensor_radius_mm=18\n";
    out+="bias_counts="+String(s.sensor.biasCounts,6)+"\nphysical_rpm="+String(s.sensor.rpm,3)+"\ntrim_period="+String(s.trim,6)+"\ntrim_led_level="+String(ash::clamp((s.trim-0.9f)*45.0f,0,9),3)+"\n";
    out+="trim_saved="+String(s.trimSaved)+"\ntrim_save_failed="+String(s.trimSaveFailed)+"\n";
    out+="validated_RC_frames="+String(s.rc.frames)+"\nRC_crc_errors="+String(s.rc.crcErrors)+"\nlink_quality="+String(s.rc.lq)+"\nlink_statistics_seen="+String(s.rc.linkSeen)+"\n";
    out+="sensor_ready_count="+String(s.sensor.fresh)+"\nsensor_overruns="+String(s.sensor.overruns)+"\noutput_iterations="+String(s.output.ticks)+"\n";
    out+="output_errors="+String(s.output.errors)+"\nmax_output_gap_us="+String(s.output.maxGap)+"\nnotification_skips="+String(s.output.skipped)+"\n";
    out+="controller_loss_timeout_us=5000000\ncontroller_last_live_age_us="+String(ash::age(uint32_t(esp_timer_get_time()),s.rc.liveTime))+"\nreceiver_reported_loss="+String(s.rc.reportedLoss)+"\n";
    out+="left_output_ready="+String(s.output.leftReady)+"\nright_output_ready="+String(s.output.rightReady)+"\nleft_recoveries="+String(s.output.leftRecoveries)+"\nright_recoveries="+String(s.output.rightRecoveries)+"\noutput_busy_retries="+String(s.output.busy)+"\ndriving_gaps_over_1ms="+String(s.output.late)+"\nworker_allocation_retries="+String(taskRetries)+"\n";
    out+="estimated_rpm="+String(s.output.rpm,3)+"\nheading_quality="+String(ash::qualityName(s.output.quality))+"\nsensor_accepted="+String(s.sensor.accepted)+"\nsensor_rejected="+String(s.sensor.rejected)+"\nsensor_rail_samples="+String(s.sensor.rails)+"\nsensor_recoveries="+String(s.sensor.recoveries)+"\n";
    out+="translation_profile="+String(ash::profileName(s.tuning.profile))+"\ntranslation_gain="+String(s.tuning.gain,3)+"\ntranslation_offset_deg="+String(s.tuning.phaseOffset,3)+"\ntranslation_lead_ms="+String(s.tuning.leadMs,3)+"\nlearned_rpm_per_command="+String(s.output.learned,3)+"\n";
    out+="diagnostic_leds="+String(s.tuning.diagnosticLeds)+"\ncapture_enabled="+String(s.tuning.capture)+"\ncapture_capacity=256\ncapture_interval_us=20000\nsettings_save_failed="+String(settingsSaveFailed)+"\n";
    out+="ota_status="+String(otaStatus)+"\nOTA_slots_valid="+String(slotsOK())+"\nreset_reason_code="+String(unsigned(resetReason))+"\n";
    auto running=esp_ota_get_running_partition();auto next=esp_ota_get_next_update_partition(nullptr);
    out+="running_slot="+String(running?running->label:"unavailable")+"\nnext_slot="+String(next?next->label:"unavailable")+"\nnext_slot_bytes="+String(next?next->size:0)+"\n";
    out+="free_heap="+String(ESP.getFreeHeap())+"\ndownload_failures="+String(downloadFailures)+"\nAP_start_failures="+String(apFailures)+"\nzero_ack_timeouts="+String(zeroAckTimeouts)+"\n";
    out+="driver_acceptance_is_not_ESC_acknowledgement=true\nno_ESC_settings_saved=true\n";return out;
}
#include "RunExport.inc"

static bool beginExport(bool requireFrozen=false) {
    portENTER_CRITICAL(&stateMux);bool ok=!state.output.armed && !state.updating && !state.saving && (!requireFrozen || state.runFrozen);
    if(ok)state.exporting=true;portEXIT_CRITICAL(&stateMux);
    if(!ok){baselineServer.send(409,"text/plain","Stop with CH5 HIGH before downloading.");return false;}
    if(!waitForZero()){portENTER_CRITICAL(&stateMux);state.exporting=false;portEXIT_CRITICAL(&stateMux);baselineServer.send(503,"text/plain","Waiting for inhibited motor owner.");return false;}
    return true;
}
static void endExport(){portENTER_CRITICAL(&stateMux);state.exporting=false;portEXIT_CRITICAL(&stateMux);}
#include "Maintenance.inc"
