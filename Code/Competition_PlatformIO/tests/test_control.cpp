#include "ControlCore.h"
#include "SensorHealth.h"
#include "RcObserver.h"
#include "RunCsv.h"
#include <cassert>
#include <vector>
#include <algorithm>
#include <iostream>
using namespace ash;
static SafetyInputs healthy(uint32_t now) {
    SafetyInputs i;i.ready=true;i.heartbeat=now;i.rc.seen=i.rc.linkSeen=true;i.rc.lq=100;
    i.rc.time=i.rc.linkTime=now;i.sensor.time=now;i.sensor.calibrated=true;return i;
}
static void arm(ArmGate& gate,SafetyInputs& i,uint32_t now) {
    i.rc.ch[4]=100;assert(!gate.step(i,now));i.rc.ch[4]=0;assert(gate.step(i,now));
}
static std::vector<uint8_t> channelsFrame(uint16_t value) {
    std::vector<uint8_t> f(26,0);f[0]=0xc8;f[1]=24;f[2]=0x16;
    for(unsigned ch=0;ch<16;++ch)for(unsigned b=0;b<11;++b)f[3+(ch*11+b)/8]|=((value>>b)&1)<<((ch*11+b)%8);
    f[25]=RcObserver::crc(f.data()+2,23);return f;
}
int main() {
    // The 1.0 summary had completed calibration plus a latched failure. Cover both
    // possible sources: startup scheduling delay and transient register readback.
    SensorCalibration slow(1000);
    for(unsigned n=1;n<=400;++n)slow.sample(-16,1000+n*7000);
    assert(slow.done && !slow.expired(4000000) && slow.elapsed==2800000 && slow.bias==-16);
    SensorCalibration missing(1000);
    for(unsigned n=1;n<400;++n)missing.sample(4,1000+n*2500);
    assert(!missing.expired(1000+SensorCalibration::TIMEOUT_US-1));
    assert(missing.expired(1000+SensorCalibration::TIMEOUT_US));
    missing.sample(4,1000+SensorCalibration::TIMEOUT_US+1);
    assert(!missing.done && missing.count==399); // Cannot later turn a timeout into calibrated=true.
    SensorCalibration calibrationWrap(0xfffffff0);
    for(unsigned n=1;n<=400;++n)calibrationWrap.sample(-17,uint32_t(0xfffffff0+n*2500));
    assert(calibrationWrap.done && calibrationWrap.elapsed==1000000 && calibrationWrap.bias==-17);
    SensorRegisters regs,bad;uint32_t badReads=0;unsigned reads=0,pauses=0;
    auto transient=[&](uint8_t address)->uint8_t {
        unsigned attempt=reads++/3;
        return attempt==0?0xff:(address==0x0f?0x32:(address==0x20?0x37:0xb0));
    };
    assert(verifySensorRegisters(transient,[&](){++pauses;},regs,bad,badReads));
    assert(regs.valid() && reads==6 && pauses==1 && badReads==1 && bad.who==0xff);
    reads=pauses=badReads=0;
    auto disconnected=[&](uint8_t)->uint8_t{++reads;return 0xff;};
    assert(!verifySensorRegisters(disconnected,[&](){++pauses;},regs,bad,badReads));
    assert(reads==9 && pauses==2 && badReads==3 && !regs.valid());
    auto wrongConfig=[](uint8_t address)->uint8_t{return address==0x0f?0x32:(address==0x20?0x27:0x90);};
    assert(!verifySensorRegisters(wrongConfig,[](){},regs,bad,badReads));
    assert(bad.who==0x32 && bad.ctrl1==0x27 && bad.ctrl4==0x90);
    auto failedStartup=healthy(1000);failedStartup.sensor.failed=true;ArmGate failedGate;
    assert(!failedGate.step(failedStartup,1000) && failedGate.stop==Stop::INIT_FAILED);
    assert(fabsf(rpmFromG(7.8627f)-625)<0.1f);
    assert(fabsf(rpmFromG(503.21f)-5000)<0.1f);
    assert(SENSOR_G_PER_COUNT==0.195f);
    assert(channelPercent(172)==0 && channelPercent(1811)==100 && channelPercent(992)==50);
    Estimator estimator;estimator.sample(0,1000);estimator.sample(10,21000);
    assert(fabsf(estimator.filtered-6.3212056f)<0.0001f);
    Phase phase;phase.reset(1000);phase.step(97000,625);assert(fabsf(phase.degrees)<0.001f);
    phase.step(98000,1250);assert(fabsf(phase.degrees-7.5f)<0.001f); // Speed change does not re-anchor phase.
    phase.reset(0xfffffff0);phase.step(16,625);assert(fabsf(phase.degrees-0.12f)<0.001f);
    Trim trim;assert(fabsf(trim.level()-4.5f)<0.00001f);
    assert(trimPixel(4.5,3)==85 && trimPixel(4.5,4)==43 && trimPixel(4.5,5)==0);
    assert(trimPixel(1.2,0)==85 && trimPixel(1.2,1)==17 && trimPixel(1.2,2)==0);
    assert(trim.adjust(100,1));float first=trim.period;assert(!trim.adjust(100,2) && trim.period==first);
    trim.adjust(50,3);assert(trim.adjust(0,4));assert(fabsf(trim.period-1)<0.00001f);
    for(int n=0;n<100;++n){trim.adjust(50,n*2);trim.adjust(100,n*2+1);}assert(trim.period==1.1f);
    assert(trim.visible(1000));trim.adjust(50,1001);assert(!trim.visible(1500200));
    auto record=trimRecord(1.004f);assert(trimValid(record));record.period=1.18f;record.checksum=trimChecksum(record);assert(!trimValid(record));
    record=trimRecord(1.004f);record.checksum^=1;assert(!trimValid(record));record={};assert(!trimValid(record));
    for(int throttle=11;throttle<=100;++throttle)for(int stick=0;stick<=100;++stick)for(int reverse=0;reverse<2;++reverse)
        for(int degrees=0;degrees<360;degrees+=3) {
            Mix m=melty(throttle,stick,reverse,5000,float(degrees));
            assert(abs(m.left)<=999 && abs(m.right)<=999);
            assert(m.left+m.right==2*m.base);
            assert(reverse?(m.left<=0 && m.right<=0):(m.left>=0 && m.right>=0));
        }
    assert(melty(17,60,false,625,180).delta==0);
    assert(abs(melty(17,61,false,625,180).delta)<=4);
    auto forward=melty(17,66,false,625,180),backward=melty(17,34,false,625,180);
    assert(forward.delta==-backward.delta && forward.delta>0);
    assert(melty(17,66,true,625,180).delta<0 && melty(17,34,true,625,180).delta>0);
    auto full=melty(100,100,false,5000,180);assert(full.base==499 && full.left==998 && full.right==0);
    auto opposite=melty(100,100,false,5000,0);
    assert(opposite.base==full.base && opposite.left==full.right && opposite.right==full.left);
    // Under a linear differential-force model, the opposite half-turn adds the
    // same world-direction component. This is not a prediction of motor force.
    double fullProjection=0,halfProjection=0,crossProjection=0;
    for(unsigned degrees=0;degrees<360;++degrees) {
        float angle=degrees*0.01745329252f,wave=-cosf(angle);
        auto m=melty(50,100,false,2000,float(degrees));
        auto other=melty(50,100,false,2000,float(degrees+180));
        assert(abs(m.delta+other.delta)<=1);
        fullProjection+=m.delta*wave;halfProjection+=fmaxf(0,float(m.delta))*wave;
        crossProjection+=m.delta*sinf(angle);
    }
    assert(fabs(fullProjection/halfProjection-2)<0.001 && fabs(crossProjection)<1);
    CaptureWindow capture;assert(!capture.keep(Drive::STOP,100));
    unsigned kept=0;
    for(unsigned n=0;n<2400;++n)kept+=capture.keep(n<400?Drive::TRANSLATE:Drive::STOP,1000+n*10000);
    assert(kept==500); // Four seconds driving + one-second tail survives 20 seconds idle.
    assert(capture.keep(Drive::STOP,25000000,true)); // Final kill/failsafe row is always retained.
    assert(capture.keep(Drive::SPIN,26000000)); // Driving again resumes recording.
    CaptureWindow captureWrap;assert(captureWrap.keep(Drive::SPIN,0xfffffff0));
    assert(captureWrap.keep(Drive::STOP,16) && !captureWrap.keep(Drive::STOP,1000000));
    auto i=healthy(1000);ArmGate gate;i.rc.ch[4]=0;assert(!gate.step(i,1000));arm(gate,i,1000);assert(gate.generation==1);
    i.rc.ch[2]=17;i.rc.time=1000;assert(!gate.step(i,101001) && gate.stop==Stop::RC_STALE);
    i=healthy(102000);i.rc.ch[4]=0;assert(!gate.step(i,102000));arm(gate,i,102000);assert(gate.generation==2);
    i.rc.lq=0;assert(!gate.step(i,102001) && gate.stop==Stop::LINK_LOST);
    i=healthy(103000);arm(gate,i,103000);i.updating=true;assert(!gate.step(i,103001) && gate.stop==Stop::OTA);
    i.updating=false;i=healthy(104000);i.rc.ch[4]=0;assert(!gate.step(i,104000));
    arm(gate,i,104000);i.sensor.rail=true;assert(!gate.step(i,104001) && gate.stop==Stop::SENSOR_RAIL);
    i=healthy(105000);arm(gate,i,105000);i.heartbeat=1;assert(!gate.step(i,105001) && gate.stop==Stop::MAIN_STALE);
    i=healthy(206000);arm(gate,i,206000);i.sensor.time=1;assert(!gate.step(i,206001) && gate.stop==Stop::SENSOR_STALE);
    i=healthy(307000);arm(gate,i,307000);i.rc.ch[4]=100;assert(!gate.step(i,307001) && gate.stop==Stop::KILL);
    i=healthy(308000);arm(gate,i,308000);i.exporting=true;assert(!gate.step(i,308001));
    i=healthy(309000);arm(gate,i,309000);i.saving=true;assert(!gate.step(i,309001));
    i=healthy(310000);arm(gate,i,310000);i.fatal=true;assert(!gate.step(i,310001));
    i=healthy(400000);ArmGate notReady;i.ready=false;assert(!notReady.step(i,400000));i.rc.ch[4]=0;assert(!notReady.step(i,400001));
    for(int mode=0;mode<3;++mode) {
        i=healthy(500000);ArmGate loss;arm(loss,i,500000);
        if(mode==0)i.rc.ch[2]=17;
        else {i.rc.ch[0]=100;if(mode==2){i.rc.ch[5]=100;i.rc.ch[1]=100;}}
        assert(!loss.step(i,600001) && loss.stop==Stop::RC_STALE);
        i=healthy(601000);i.rc.ch[4]=0;i.rc.ch[2]=17;assert(!loss.step(i,601000));
        i.rc.ch[2]=0;assert(!loss.step(i,601000));arm(loss,i,601000);
    }
    i=healthy(700000);ArmGate reportedLoss;arm(reportedLoss,i,700000);
    i.rc.time=i.rc.linkTime=700001;i.rc.lq=0;assert(!reportedLoss.step(i,700001)); // Held but valid channels cannot override reported RF loss.
    i=healthy(20);i.rc.time=i.rc.linkTime=i.sensor.time=i.heartbeat=0xfffffff0;ArmGate rollover;arm(rollover,i,20);
    RcObserver observer;auto f=channelsFrame(992);for(auto b:f)observer.feed(b,1000);
    assert(observer.frames==1 && observer.channels[0]==992 && observer.channels[15]==992);
    f[25]^=1;for(auto b:f)observer.feed(b,2000);assert(observer.frames==1 && observer.lastRcUs==1000 && observer.crcErrors==1);
    std::vector<uint8_t> stats(14,0);stats[0]=0xc8;stats[1]=12;stats[2]=0x14;stats[5]=100;stats[13]=RcObserver::crc(stats.data()+2,11);
    for(auto b:stats)observer.feed(b,3000);assert(observer.frames==1 && observer.lastRcUs==1000 && observer.linkFrames==1 && observer.lastLinkUs==3000);
    stats[5]=0;stats[13]=RcObserver::crc(stats.data()+2,11);for(auto b:stats)observer.feed(b,4000);assert(observer.linkLost);
    i=healthy(2000000);i.rc.ch[4]=0;i.rc.linkTime=1;assert(fault(i,2000000)==Stop::LINK_LOST);
    Rc r;r.ch[0]=100;r.ch[4]=0;auto tank=ground(r);assert(tank.mode==Drive::TANK && tank.left==260 && tank.right==260);
    r.ch[5]=100;r.ch[1]=100;auto unstick=ground(r);assert(unstick.mode==Drive::UNSTICK && unstick.left==120 && unstick.right==-120);
    char line[640];BaseRow row{};row.sensorUs=uint32_t(-12);row.trim=1;
    int n=formatBaselineRow(line,sizeof(line),row);assert(n>0 && size_t(n)<sizeof(line));assert(std::count(line,line+n,',')==31);
    std::cout<<"PASS: delayed/failed sensor calibration, transient/persistent readback, sensor units/filter, continuous phase/rollover, trim/LED/settings, exhaustive bounded mixer, receiver parser and competition stop/rearm guards.\n";
}
