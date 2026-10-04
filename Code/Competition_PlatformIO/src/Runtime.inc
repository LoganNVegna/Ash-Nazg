// Cooperative steps allow the main loop to cover a failed worker allocation.
// Once created, each worker is the sole owner of its peripheral/state machine.
static void receiverTick() {
    static bool initialized=false;static uint32_t retryAt=0,lastTraffic=0;
    uint32_t now=uint32_t(esp_timer_get_time());
    if(initialized && (!bool(Serial1) || ash::age(now,lastTraffic)>1000000)) {
        Serial1.end();rcObserver.resetParser();initialized=false;retryAt=now;
    }
    if(!initialized) {
        if(retryAt && int32_t(now-retryAt)<0)return;
        Serial1.setRxBufferSize(512);Serial1.begin(416666,SERIAL_8N1,16,5);
        initialized=bool(Serial1);retryAt=now+500000;lastTraffic=now;
        if(!initialized){Serial1.end();return;}
    }
    for(unsigned n=0;n<128 && Serial1.available();++n) {
        now=uint32_t(esp_timer_get_time());uint32_t links=rcObserver.linkFrames;
        bool controls=rcObserver.feed(uint8_t(Serial1.read()),now);
        if(!controls && links==rcObserver.linkFrames)continue;
        lastTraffic=now;
        portENTER_CRITICAL(&stateMux);
        ash::Rc& r=state.rc;r.frames=rcObserver.frames;r.crcErrors=rcObserver.crcErrors;
        if(links!=rcObserver.linkFrames){r.linkTime=now;r.linkSeen=true;r.lq=rcObserver.lq;r.reportedLoss=rcObserver.linkLost;}
        if(controls) {
            r.time=now;r.seen=true;
            if(!r.reportedLoss) {
                for(unsigned ch=0;ch<8;++ch)r.ch[ch]=ash::channelPercent(rcObserver.channels[ch]);
                r.liveTime=now;r.liveSeen=true;++r.sequence;
            }
        }
        portEXIT_CRITICAL(&stateMux);
    }
    portENTER_CRITICAL(&stateMux);state.rc.crcErrors=rcObserver.crcErrors;portEXIT_CRITICAL(&stateMux);
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
static void sensorTick() {
    static bool initialized=false,verified=false,wasStopped=false,wasRejected=false;
    static uint32_t checkAt=0,start=0;static ash::Sensor s;
    static ash::SensorRegisters registers,lastBad;static uint32_t badReads=0;
    static ash::SensorFault fault=ash::SensorFault::NONE;static uint32_t faultAt=0;
    static ash::SensorCalibration calibration(0);static ash::RadialFilter filter;
    uint32_t now=uint32_t(esp_timer_get_time());Shared shared=snapshot();
    if(!initialized) {
        pinMode(8,OUTPUT);digitalWrite(8,HIGH);SPI.begin(36,37,35,8);
        s.biasCounts=shared.sensor.biasCounts;s.biasKnown=shared.sensor.biasKnown;initialized=true;start=now;
        calibration=ash::SensorCalibration(now);
    }
    if(!checkAt || ash::age(now,checkAt)>=500000) {
        checkAt=now;bool previous=verified;
        verified=ash::verifySensorRegisters(sensorReg,[]{vTaskDelay(1);},registers,lastBad,badReads);
        if(!verified && registers.who==0x32) {
            // Restore only documented configuration. This SPI owner never blocks output.
            sensorWrite(0x20,0x37);sensorWrite(0x23,0xb0);
        }
        if(verified && !previous){++s.recoveries;fault=ash::SensorFault::NONE;}
        if(!verified){fault=ash::SensorFault::CONFIG_REGISTERS;faultAt=now;}
        s.who=registers.who;s.ctrl1=registers.ctrl1;s.ctrl4=registers.ctrl4;
    }
    s.failed=!verified;
    bool stationary=!shared.output.armed && ash::neutral(shared.rc) && shared.output.rpm<100;
    if(stationary && !wasStopped && !s.calibrated){calibration=ash::SensorCalibration(now);start=now;}
    wasStopped=stationary;
    if(!verified) {
        portENTER_CRITICAL(&stateMux);state.sensor=s;state.sensorFault=fault;state.sensorFaultUs=faultAt;
        state.sensorRegisters=registers;state.sensorLastBad=lastBad;state.sensorBadReads=badReads;portEXIT_CRITICAL(&stateMux);return;
    }
    uint8_t status=sensorReg(0x27);++s.polls;s.status=status;
    if(status!=0xff && (status&8)) {
        int16_t xyz[3];sensorXYZ(xyz);s.time=uint32_t(esp_timer_get_time());++s.fresh;
        s.x=xyz[0];s.y=xyz[1];s.z=xyz[2];if(status&128)++s.overruns;
        s.rail=abs(s.y)>=2000;if(s.rail)++s.rails;
        // No moving bias calibration. An interrupted window starts over at rest.
        if(!s.calibrated) {
            if(stationary && !s.rail && abs(s.y)<100 && abs(s.x)<100 && abs(s.z)<100) {
                calibration.sample(s.y,s.time);
                if(calibration.done){s.biasCounts=calibration.bias;s.calibrated=s.biasKnown=true;filter={};}
            } else if(!stationary)calibration=ash::SensorCalibration(s.time);
            if(calibration.expired(now)){fault=ash::SensorFault::CALIBRATION_TIMEOUT;faultAt=now;calibration=ash::SensorCalibration(now);}
        }
        s.radialG=(s.y-s.biasCounts)*ash::SENSOR_G_PER_COUNT;
        bool accepted=filter.sample(s,s.time,shared.tuning.robustFilter);
        s.impact=!accepted;
        if(accepted){s.filteredG=filter.ema.filtered;s.rpm=filter.lastRpm;s.acceptedTime=s.time;++s.accepted;if(s.calibrated)fault=ash::SensorFault::NONE;}
        else ++s.rejected;
        if(accepted!=!wasRejected) {
            portENTER_CRITICAL(&stateMux);events.add(s.time,accepted?ash::EventKind::SENSOR_RECOVER:ash::EventKind::SENSOR_REJECT,s.y);portEXIT_CRITICAL(&stateMux);
        }
        wasRejected=!accepted;
    }
    portENTER_CRITICAL(&stateMux);state.sensor=s;state.sensorFault=fault;state.sensorFaultUs=faultAt;
    state.sensorRegisters=registers;state.sensorLastBad=lastBad;state.sensorBadReads=badReads;
    state.calibrationCount=calibration.count;state.calibrationUs=ash::age(now,start);portEXIT_CRITICAL(&stateMux);
}

struct EscPort {
    DShotESC& esc;gpio_num_t pin;rmt_channel_t channel;bool installed=false;
    EscPort(DShotESC& driver,gpio_num_t gpio,rmt_channel_t rmt):esc(driver),pin(gpio),channel(rmt){}
    static ash::Io result(esp_err_t error){return error==ESP_OK?ash::Io::OK:error==ESP_ERR_TIMEOUT?ash::Io::BUSY:ash::Io::ERROR;}
    ash::Io install(bool) {
        // Also clear a partially installed driver left by a failed allocation.
        // All our writes are nonblocking, so IDF uninstall has no TX wait.
        rmt_driver_uninstall(channel);
        auto e=esc.install(pin,channel,600000UL,3);installed=e==ESP_OK;return result(e);
    }
    void recover(){if(installed){rmt_tx_stop(channel);esc.uninstall();installed=false;}pinMode(pin,OUTPUT);digitalWrite(pin,LOW);}
    ash::Io stop(){return result(esc.writeData(0,false));}
    ash::Io direction(){return result(esc.sendStartupCommand(DSHOT_CMD::SPIN_DIRECTION_NORMAL));}
    ash::Io mode(){return result(esc.sendStartupCommand(DSHOT_CMD::MODE_3D_ON));}
    ash::Io throttle(int16_t value){return result(esc.sendThrottle3D(value));}
};
static void motorWake(void*){if(motorTaskHandle)xTaskNotifyGive(motorTaskHandle);}
static void motorTick() {
    static EscPort right{escR,GPIO_NUM_18,RMT_CHANNEL_3},left{escL,GPIO_NUM_17,RMT_CHANNEL_2};
    static ash::ChannelRecovery cr,cl;static ash::ArmGate gate;static ash::Phase phase;
    static ash::RateTracker tracker;static ash::SpinRamp ramp;static ash::Timing timing;
    static uint32_t previousOutput=0;static bool haveOutput=false,spinDetected=false;
    uint32_t now=uint32_t(esp_timer_get_time());Shared s=snapshot();
    uint32_t interval=gate.armed?166:1000;
    if(haveOutput && ash::age(now,previousOutput)<interval)return;
    previousOutput=now;haveOutput=true;
    ash::SafetyInputs inputs;inputs.rc=s.rc;inputs.ready=cl.ready()||cr.ready();inputs.updating=s.updating;
    inputs.exporting=s.exporting||s.networkBusy;inputs.saving=s.saving;
    bool wasArmed=gate.armed;bool permit=gate.step(inputs,now);
    bool reverse=s.rc.ch[5]>50;float spin=ramp.step(permit?s.rc.ch[2]:0,reverse,now);
    float rpm=tracker.step(s.sensor,now,spin,s.tuning.rpmPerCommand);
    if(!spinDetected && rpm>=450)spinDetected=true;
    if(spinDetected && rpm<=350 && tracker.quality==ash::Quality::MEASURED)spinDetected=false;
    float heading=1+(s.rc.ch[0]-50)*0.001f*(reverse?-1:1);
    float rate=rpm/(s.trim*heading);phase.step(now,rate);
    float drivePhase=ash::modulationPhase(phase.degrees,rpm,reverse,s.tuning);
    ash::Mix mix;
    if(permit) {
        if(s.rc.ch[2]>10) {
            // Ramp crosses zero on reversal; don't jump straight to reverse base.
            bool effectiveReverse=spin<0;
            drivePhase=ash::modulationPhase(phase.degrees,rpm,effectiveReverse,s.tuning);
            int translate=(spinDetected || tracker.quality!=ash::Quality::MEASURED)?s.rc.ch[1]:50;
            ash::Tuning active=s.tuning;
            // Fade authority through spin-up/reversal, rather than stepping it on.
            active.gain*=fminf(1,fabsf(spin)/fminf(999,float(s.rc.ch[2]*10)));
            mix=ash::mixMelty(spin,translate,effectiveReverse,drivePhase,active);
        } else mix=ash::ground(s.rc);
    }
    uint32_t leftBefore=cl.recoveries,rightBefore=cr.recoveries;
    bool rightSent=cr.step(right,now,mix.right);delayMicroseconds(25);
    uint32_t leftNow=uint32_t(esp_timer_get_time());bool leftSent=cl.step(left,leftNow,mix.left);
    uint32_t done=uint32_t(esp_timer_get_time());phase.step(done,rate);
    Output out=s.output;out.wanted=mix;out.left=cl.submitted;out.right=cr.submitted;
    if(rightSent && leftSent){out.appliedPhase=drivePhase;out.pairTime=now;}
    out.leftTime=cl.submittedAt;out.rightTime=cr.submittedAt;out.leftUnavailable=!left.installed;out.rightUnavailable=!right.installed;
    out.phase=phase.degrees;out.drivePhase=drivePhase;out.rateRpm=rate;out.rpm=rpm;out.quality=tracker.quality;
    out.learned=tracker.learned;out.time=done;++out.ticks;out.computeUs=uint16_t(min(ash::age(done,now),65535u));
    out.armed=gate.armed;out.generation=gate.generation;out.stop=gate.stop;out.stopUs=gate.stoppedAt;
    out.escReady=cl.ready()||cr.ready();out.leftReady=cl.ready();out.rightReady=cr.ready();
    out.pinsLow=!left.installed&&!right.installed;out.errors=cl.errors+cr.errors;out.busy=cl.busy+cr.busy;
    out.leftRecoveries=cl.recoveries;out.rightRecoveries=cr.recoveries;
    bool driving=permit && mix.mode!=ash::Drive::STOP;
    uint32_t gap=timing.step(done,driving);out.maxGap=timing.maxGap;out.late=timing.late;
    portENTER_CRITICAL(&stateMux);
    if(wasArmed!=gate.armed)events.add(now,gate.armed?ash::EventKind::ARM:ash::EventKind::STOP,unsigned(gate.stop));
    if(cl.recoveries!=leftBefore)events.add(done,ash::EventKind::LEFT_RECOVER,cl.errors);
    if(cr.recoveries!=rightBefore)events.add(done,ash::EventKind::RIGHT_RECOVER,cr.errors);
    if(driving && gap>1000 && (gap>state.output.maxGap || gap>5000))events.add(done,ash::EventKind::OUTPUT_GAP,gap);
    if(gate.armed)state.runFrozen=false;
    state.output=out;portEXIT_CRITICAL(&stateMux);
}

static void ledTick() {
    Shared s=snapshot();uint32_t now=uint32_t(esp_timer_get_time());topStrip.clear();
    if(s.trimVisible)for(unsigned p=0;p<9;++p){uint8_t b=ash::trimPixel(ash::clamp((s.trim-0.9f)*45,0,9),p);topStrip.setPixelColor(p,b,b,b);}
    else if(!s.output.armed)topStrip.fill(topStrip.Color(0,0,s.updating?70:25));
    else {
        float phase=ash::wrap(s.output.phase+ash::age(now,s.output.time)*s.output.rateRpm*0.000006f);
        bool marker=ash::markerWindow(phase,0,s.output.rateRpm);
        if(s.tuning.diagnosticLeds) {
            if(marker)topStrip.setPixelColor(0,85,85,85);
            auto commandColor=[&](int16_t command){uint8_t b=uint8_t(ash::clamp(fabsf(float(command))*85/999,0,85));return command>=0?topStrip.Color(0,b,0):topStrip.Color(b,0,0);};
            topStrip.setPixelColor(1,commandColor(s.output.left));topStrip.setPixelColor(2,commandColor(s.output.right));
            float p=s.output.appliedPhase;
            uint8_t amplitude=uint8_t(ash::clamp(s.output.wanted.strength*85,0,85));
            if(s.output.wanted.mode==ash::Drive::TRANSLATE && s.output.left!=s.output.right && ash::age(now,s.output.pairTime)<2000) {
                if(ash::markerWindow(p,180,s.output.rateRpm))topStrip.setPixelColor(3,0,amplitude,0);
                else if((s.tuning.profile==ash::Profile::STRONG_FULL || s.tuning.profile==ash::Profile::BOUNDED_FULL) && ash::markerWindow(p,0,s.output.rateRpm))topStrip.setPixelColor(3,0,amplitude,amplitude);
            }
            topStrip.setPixelColor(4,s.output.quality==ash::Quality::MEASURED?topStrip.Color(0,0,10):topStrip.Color(18,8,0));
            if(!s.output.leftReady)topStrip.setPixelColor(5,25,0,0);
            if(!s.output.rightReady)topStrip.setPixelColor(6,25,0,0);
        } else {
            if(marker)topStrip.fill(topStrip.Color(85,85,85));
            // Two phase-coded events, rather than green over almost a whole turn.
            else if(s.output.wanted.mode==ash::Drive::TRANSLATE) {
                float p=s.output.appliedPhase;
                bool freshPair=ash::age(now,s.output.pairTime)<2000 && s.output.left!=s.output.right;
                if(freshPair && ash::markerWindow(p,180,s.output.rateRpm))topStrip.setPixelColor(3,0,30,0);
                else if(freshPair && (s.tuning.profile==ash::Profile::STRONG_FULL || s.tuning.profile==ash::Profile::BOUNDED_FULL) && ash::markerWindow(p,0,s.output.rateRpm))topStrip.setPixelColor(3,0,20,20);
            }
            if(!marker && s.output.quality!=ash::Quality::MEASURED)topStrip.setPixelColor(8,10,5,0);
        }
    }
    topStrip.show();
}
static void receiverTask(void*){for(;;){receiverTick();vTaskDelay(1);}}
static void sensorTask(void*){for(;;){sensorTick();vTaskDelay(1);}}
static void ledTask(void*){for(;;){ledTick();vTaskDelay(1);}}
static void motorTask(void*) {
    esp_timer_create_args_t args{};args.callback=motorWake;args.dispatch_method=ESP_TIMER_TASK;args.name="motor_wake";args.skip_unhandled_events=true;
    if(esp_timer_create(&args,&motorTimer)==ESP_OK) {
        if(esp_timer_start_periodic(motorTimer,166)!=ESP_OK){esp_timer_delete(motorTimer);motorTimer=nullptr;}
    }
    for(;;) {
        // Finite wait remains functional if the periodic wake source disappears.
        if(motorTimer)ulTaskNotifyTake(pdTRUE,1);else vTaskDelay(1);
        motorTick();
    }
}
