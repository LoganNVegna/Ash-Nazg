static void saveSettings() {
    static uint32_t lastAttempt=0;static bool attempted=false;
    Shared s=snapshot();
    if(s.output.armed || s.updating || s.exporting || s.networkBusy || !zeroAcknowledged())return;
    bool biasDirty=s.sensor.calibrated && (!s.biasSaved || fabsf(s.savedBias-s.sensor.biasCounts)>0.05f);
    if(!trim.dirty && !s.tuningDirty && !biasDirty)return;
    if(attempted && uint32_t(millis()-lastAttempt)<1000)return;
    portENTER_CRITICAL(&stateMux);
    bool allowed=!state.output.armed && !state.updating && !state.exporting && !state.networkBusy;
    if(allowed)state.saving=true;portEXIT_CRITICAL(&stateMux);if(!allowed)return;
    attempted=true;lastAttempt=millis();
    bool trimOK=!trim.dirty,tuningOK=!s.tuningDirty,biasOK=!biasDirty;
    if(preferences.begin("ashnazg",false)) {
        if(trim.dirty){auto record=ash::trimRecord(trim.period);trimOK=preferences.putBytes("trim-v1",&record,sizeof(record))==sizeof(record);}
        if(s.tuningDirty){auto record=ash::tuningRecord(s.tuning);tuningOK=preferences.putBytes("drive-v2",&record,sizeof(record))==sizeof(record);}
        if(biasDirty)biasOK=preferences.putFloat("bias-counts",s.sensor.biasCounts)==sizeof(float);
        preferences.end();
    }
    settingsSaveFailed=!(trimOK&&tuningOK&&biasOK);if(trimOK)trim.dirty=false;
    portENTER_CRITICAL(&stateMux);state.saving=false;state.trimSaved=!trim.dirty;state.trimSaveFailed=settingsSaveFailed;
    if(tuningOK)state.tuningDirty=false;
    if(biasOK && biasDirty){state.biasSaved=true;state.savedBias=s.sensor.biasCounts;}
    portEXIT_CRITICAL(&stateMux);
}
static bool parseFloatArg(const char* name,float& value) {
    if(!baselineServer.hasArg(name))return false;
    String arg=baselineServer.arg(name);char* end=nullptr;
    if(!arg.length())return false;value=strtof(arg.c_str(),&end);
    return end!=arg.c_str() && *end=='\0' && isfinite(value);
}
static String tuningPage() {
    Shared s=snapshot();auto t=s.tuning;
    String html=String("<!doctype html><meta name='viewport' content='width=device-width'><title>AshNazg settings</title><h1>")+BUILD_NAME+"</h1><p>Stop with CH5 HIGH before changing settings. Save, close this page, then cycle CH5 HIGH to LOW.</p><form method='post' action='/tuning'><label>Translation <select name='profile'>";
    const char* labels[]={"Stronger: half turn","Stronger: both halves","Previous bounded mixer","V5 reference"};
    for(unsigned n=0;n<4;++n)html+="<option value='"+String(n)+"'"+(unsigned(t.profile)==n?" selected":"")+">"+labels[n]+"</option>";
    html+="</select></label><p><label>Translation strength <input name='gain' type='number' min='0' max='1.5' step='0.05' value='"+String(t.gain,2)+"'></label></p>";
    html+="<p><label>Translation angle (degrees) <input name='offset' type='number' min='-180' max='180' step='1' value='"+String(t.phaseOffset,1)+"'></label></p>";
    html+="<p><label>Motor timing lead (milliseconds) <input name='lead' type='number' min='-20' max='20' step='0.5' value='"+String(t.leadMs,2)+"'></label></p>";
    html+="<details><summary>Prediction and diagnostics</summary><p><label>Fallback body RPM per command <input name='model' type='number' min='0.1' max='20' step='0.1' value='"+String(t.rpmPerCommand,2)+"'></label></p>";
    html+="<p><label><input name='leds' type='checkbox'"+String(t.diagnosticLeds?" checked":"")+">Diagnostic LEDs</label></p>";
    html+="<p><label><input name='capture' type='checkbox'"+String(t.capture?" checked":"")+">Optional short CSV recording</label></p>";
    html+="<p><label><input name='robust' type='checkbox'"+String(t.robustFilter?" checked":"")+">Reject impact-contaminated measurements</label></p></details><button>Save</button></form><p><a href='/'>Status</a> | <a href='/events.txt'>Recent events</a></p>";
    return html;
}
static void configureRoutes() {
    baselineServer.on("/",[] {
        if(!beginExport())return;
        String html=String("<!doctype html><meta name='viewport' content='width=device-width'><h1>")+BUILD_NAME+"</h1><p>CH5 HIGH stops and restores Wi-Fi. Five seconds without controller commands disarms; reconnect and cycle CH5 HIGH to LOW.</p><p><a href='/tuning'>Translation and LED settings</a> | <a href='/summary.txt'>Summary</a> | <a href='/events.txt'>Recent events</a> | <a href='/run.csv'>Optional CSV</a></p><pre>"+baselineSummary()+"</pre>";
        baselineServer.send(200,"text/html",html);endExport();
    });
    baselineServer.on("/tuning",HTTP_GET,[]{if(beginExport()){baselineServer.send(200,"text/html",tuningPage());endExport();}});
    baselineServer.on("/tuning",HTTP_POST,[] {
        if(!beginExport())return;
        auto t=snapshot().tuning;float profile=-1;
        bool ok=parseFloatArg("profile",profile) && profile>=0 && profile<=3 && floorf(profile)==profile &&
            parseFloatArg("gain",t.gain) && parseFloatArg("offset",t.phaseOffset) &&
            parseFloatArg("lead",t.leadMs) && parseFloatArg("model",t.rpmPerCommand);
        t.profile=ok?ash::Profile(unsigned(profile)):t.profile;t.diagnosticLeds=baselineServer.hasArg("leds");
        t.capture=baselineServer.hasArg("capture");t.robustFilter=baselineServer.hasArg("robust");
        if(!ok || !ash::validTuning(t)){baselineServer.send(400,"text/plain","Invalid setting; no changes applied.");endExport();return;}
        portENTER_CRITICAL(&stateMux);state.tuning=t;state.tuningDirty=true;portEXIT_CRITICAL(&stateMux);
        baselineServer.sendHeader("Location","/tuning");baselineServer.send(303,"text/plain","Saved in RAM; persistence retries automatically.");endExport();
    });
    baselineServer.on("/events.txt",[] {
        if(!beginExport())return;
        ash::Events<> copy;portENTER_CRITICAL(&stateMux);copy=events;portEXIT_CRITICAL(&stateMux);
        const char* names[]={"arm","stop","sensor_reject","sensor_recovered","left_output_recovered","right_output_recovered","output_gap_us"};
        String result="boot_time_us,event,value\n";
        for(unsigned n=0;n<copy.count;++n){auto e=copy.at(n);result+=String(e.time)+","+names[unsigned(e.kind)]+","+String(e.value)+"\n";}
        baselineServer.send(200,"text/plain",result);endExport();
    });
    baselineServer.on("/status",[]{if(beginExport()){baselineServer.send(200,"text/plain",baselineSummary());endExport();}});
    baselineServer.on("/summary.txt",[]{if(beginExport()){baselineServer.sendHeader("Content-Disposition","attachment; filename=AshNazg-competition-summary.txt");baselineServer.send(200,"text/plain",baselineSummary());endExport();}});
    baselineServer.on("/run.csv",[]{if(beginExport(true)){downloadBaseline();endExport();}});
    baselineServer.on("/download-check.csv",[]{if(beginExport(true)){downloadExport(true);endExport();}});
    baselineServer.on("/export.json",[]{if(beginExport(true)){exportManifest();endExport();}});
    const char* headers[]={"Range","If-Range"};baselineServer.collectHeaders(headers,2);
}
static void networkTick() {
    Shared s=snapshot();
    if(s.output.armed){stopMaintenance();return;}
    startMaintenance();if(!wifiRunning)return;
    portENTER_CRITICAL(&stateMux);bool allowed=!state.output.armed && !state.saving;
    if(allowed)state.networkBusy=true;portEXIT_CRITICAL(&stateMux);if(!allowed)return;
    // Preflight before ArduinoOTA.handle, whose library starts Update before onStart.
    if(zeroAcknowledged()){ArduinoOTA.handle();baselineServer.handleClient();}
    portENTER_CRITICAL(&stateMux);state.networkBusy=false;portEXIT_CRITICAL(&stateMux);
}
static void networkTask(void*){for(;;){networkTick();vTaskDelay(1);}}
static void ensureTasks() {
    static uint32_t lastAttempt=0;static bool attempted=false;
    if(attempted && ash::age(uint32_t(esp_timer_get_time()),lastAttempt)<1000000)return;
    attempted=true;lastAttempt=uint32_t(esp_timer_get_time());
    if(!motorTaskHandle && xTaskCreatePinnedToCore(motorTask,"motor",6144,nullptr,2,&motorTaskHandle,1)!=pdPASS)++taskRetries;
    if(!receiverTaskHandle && xTaskCreatePinnedToCore(receiverTask,"receiver",4096,nullptr,2,&receiverTaskHandle,0)!=pdPASS)++taskRetries;
    if(!networkTaskHandle && xTaskCreatePinnedToCore(networkTask,"network",8192,nullptr,1,&networkTaskHandle,0)!=pdPASS)++taskRetries;
    if(!sensorTaskHandle && xTaskCreatePinnedToCore(sensorTask,"sensor",4096,nullptr,1,&sensorTaskHandle,0)!=pdPASS)++taskRetries;
    if(!ledTaskHandle && xTaskCreatePinnedToCore(ledTask,"led",3072,nullptr,1,&ledTaskHandle,1)!=pdPASS)++taskRetries;
}
void setup() {
    pinMode(17,OUTPUT);digitalWrite(17,LOW);pinMode(18,OUTPUT);digitalWrite(18,LOW);
    resetReason=esp_reset_reason();exportId=esp_random();Serial.begin(115200);topStrip.begin();topStrip.setBrightness(255);
    topStrip.fill(topStrip.Color(0,0,33));topStrip.show();
    ArduinoOTA.setHostname("ashnazg-competition");ArduinoOTA.setPassword("admin");
    ArduinoOTA.onStart([] {
        portENTER_CRITICAL(&stateMux);state.updating=true;portEXIT_CRITICAL(&stateMux);
        strlcpy(otaStatus,"Uploading",sizeof(otaStatus));
        if(!slotsOK() || !waitForZero()){++zeroAckTimeouts;Update.abort();strlcpy(otaStatus,"Upload refused; maintenance retained",sizeof(otaStatus));}
    });
    ArduinoOTA.onProgress([](unsigned,unsigned){feedBaselineWatchdog();});
    ArduinoOTA.onError([](ota_error_t error) {
        Update.abort();snprintf(otaStatus,sizeof(otaStatus),"Upload error %u; retry available",unsigned(error));
        portENTER_CRITICAL(&stateMux);state.updating=false;portEXIT_CRITICAL(&stateMux);
    });
    configureRoutes();startMaintenance();
    ash::TrimRecord record{};ash::TuningRecord driveRecord;ash::Tuning tuning;float bias=NAN;
    bool valid=false,validDrive=false;
    if(preferences.begin("ashnazg",true)) {
        if(preferences.getBytesLength("trim-v1")==sizeof(record))valid=preferences.getBytes("trim-v1",&record,sizeof(record))==sizeof(record)&&ash::trimValid(record);
        if(preferences.getBytesLength("drive-v2")==sizeof(driveRecord))validDrive=preferences.getBytes("drive-v2",&driveRecord,sizeof(driveRecord))==sizeof(driveRecord)&&ash::decodeTuning(driveRecord,tuning);
        bias=preferences.getFloat("bias-counts",NAN);preferences.end();
    }
    if(valid)trim.period=record.period;else trim.dirty=true;
    state.trim=trim.period;state.tuning=tuning;state.tuningDirty=!validDrive;
    if(isfinite(bias) && fabsf(bias)<100){state.sensor.biasCounts=bias;state.sensor.biasKnown=true;state.savedBias=bias;state.biasSaved=true;}
    ensureTasks();
}
void loop() {
    feedBaselineWatchdog();ensureTasks();
    if(!receiverTaskHandle)receiverTick();if(!sensorTaskHandle)sensorTick();if(!motorTaskHandle)motorTick();
    uint32_t now=uint32_t(esp_timer_get_time());Shared s=snapshot();
    if(ash::liveFrame(s.rc,now) && !s.updating)trim.adjust(s.rc.ch[3],now);
    portENTER_CRITICAL(&stateMux);state.trim=trim.period;state.trimVisible=trim.visible(now);state.trimChanged=trim.changedAt;
    state.trimSaved=!trim.dirty;state.trimSaveFailed=settingsSaveFailed;portEXIT_CRITICAL(&stateMux);
    if(s.output.armed) {
        maintenance=false;
        if(s.output.generation!=captureGeneration) {
            portENTER_CRITICAL(&stateMux);
            if(state.output.armed){captureGeneration=s.output.generation;captureStart=0;rowsTotal=rowsRetained=writeIndex=0;captureWindow={};exportId=esp_random();}
            portEXIT_CRITICAL(&stateMux);
        }
        if(s.tuning.capture){if(!captureStart && s.output.wanted.mode!=ash::Drive::STOP)captureStart=now;recordRun();}
    } else {
        if(!maintenance){if(s.tuning.capture)recordRun(true);maintenance=true;}
        portENTER_CRITICAL(&stateMux);if(!state.output.armed)state.runFrozen=true;portEXIT_CRITICAL(&stateMux);
        saveSettings();if(!networkTaskHandle)networkTick();
    }
    if(!ledTaskHandle)ledTick();delay(1);
}
