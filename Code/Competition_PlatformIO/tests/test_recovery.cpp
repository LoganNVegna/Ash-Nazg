#include "RecoveryCore.h"
#include <cassert>
#include <iostream>
using namespace ash;
static void frame(SafetyInputs& i,uint32_t now,int ch5){i.rc.ch[4]=ch5;i.rc.liveTime=now;i.rc.liveSeen=true;++i.rc.sequence;}
static void arm(ArmGate& g,SafetyInputs& i,uint32_t now){i.ready=true;frame(i,now,100);assert(!g.step(i,now));frame(i,now+1,0);assert(g.step(i,now+1));}
struct FakeDriver {
    Io status=Io::OK;int installs=0,zeros=0,directions=0,modes=0,throttles=0,restarts=0;
    Io install(bool){++installs;return status;}
    void recover(){++restarts;}
    Io stop(){++zeros;return status;}
    Io direction(){++directions;return status;}
    Io mode(){++modes;return status;}
    Io throttle(int16_t){++throttles;return status;}
};
int main() {
    SafetyInputs i;ArmGate gate;arm(gate,i,1000);i.rc.ch[2]=60;
    assert(gate.step(i,5001000)); // 4,999,999 us from the last live frame.
    assert(!gate.step(i,5001001) && gate.stop==Stop::RC_STALE);
    frame(i,5002000,0);assert(!gate.step(i,5002000));
    frame(i,5003000,100);assert(!gate.step(i,5003000));
    frame(i,5004000,0);assert(gate.step(i,5004000)); // No neutral requirement on recovery.
    i.ready=false;i.saving=i.exporting=true;assert(gate.step(i,5004001));
    i.updating=true;assert(!gate.step(i,5004002));i.updating=false;i.saving=i.exporting=false;i.ready=true;
    assert(!gate.step(i,5004003));frame(i,5005000,100);assert(!gate.step(i,5005000));
    frame(i,5006000,0);assert(gate.step(i,5006000));
    frame(i,5007000,100);assert(!gate.step(i,5007000) && gate.stop==Stop::KILL);
    frame(i,5008000,100);assert(!gate.step(i,5008000));
    i.exporting=true;frame(i,5009000,0);assert(!gate.step(i,5009000));i.exporting=false;
    frame(i,5010000,0);assert(gate.step(i,5010000)); // A brief network step doesn't erase the high edge.
    SafetyInputs boot;boot.ready=true;boot.rc.ch[2]=20;ArmGate bootGate;
    frame(boot,100,100);assert(!bootGate.step(boot,100));frame(boot,101,0);assert(!bootGate.step(boot,101));
    boot.rc.ch[2]=0;arm(bootGate,boot,102);
    SafetyInputs wrap;ArmGate wrapGate;arm(wrapGate,wrap,0xfffffff0u);
    assert(wrapGate.step(wrap,uint32_t(0xfffffff1u+4999999u)));
    assert(!wrapGate.step(wrap,uint32_t(0xfffffff1u+5000000u)));
    SafetyInputs stats;ArmGate statsGate;arm(statsGate,stats,100);
    stats.rc.reportedLoss=true;stats.rc.lq=0;stats.rc.time=stats.rc.linkTime=4000000;
    assert(statsGate.step(stats,4000000)); // Held channel/stats timestamps do not refresh live controls.
    assert(!statsGate.step(stats,5000101));

    Sensor sensor;sensor.calibrated=true;sensor.accepted=1;sensor.acceptedTime=1000;sensor.rpm=625;
    RateTracker tracker;assert(tracker.step(sensor,1000,250,4)==625);
    Phase phase;phase.reset(1000);sensor.failed=true;sensor.rail=true;sensor.rpm=NAN;
    for(uint32_t n=1100;n<=97000;n+=100){float r=tracker.step(sensor,n,250,4);assert(r==625);phase.step(n,r);}
    assert(fabsf(phase.degrees)<0.01f && tracker.quality==Quality::PREDICTED);
    sensor.failed=sensor.rail=false;sensor.rpm=700;sensor.acceptedTime=98000;++sensor.accepted;
    tracker.step(sensor,98000,250,4);assert(tracker.rpm>625 && tracker.rpm<=645);
    sensor.acceptedTime=100000;++sensor.accepted;tracker.step(sensor,100000,250,4);assert(tracker.rpm>645);
    Sensor unknown;RateTracker fallback;
    for(uint32_t now=1;now<2000000;now+=1000)fallback.step(unknown,now,250,4);
    assert(fallback.rpm>800 && fallback.quality==Quality::PREDICTED);
    Sensor s;s.radialG=-8;s.y=-41;RadialFilter filter;assert(filter.sample(s,1000,true));float first=filter.lastRpm;
    s.radialG=-380;s.y=-1948;assert(!filter.sample(s,3500,true) && filter.lastRpm==first);
    s.rail=true;assert(!filter.sample(s,6000,true));s.rail=false;s.radialG=-8;s.y=-41;
    assert(filter.sample(s,8500,true));assert(fabsf(filter.lastRpm-first)<0.1f);
    s.radialG=-40;s.y=-205;assert(!filter.sample(s,11000,true));assert(!filter.sample(s,13500,true));assert(filter.sample(s,16000,true));
    assert(filter.lastRpm>first && filter.lastRpm-first<=150.1f);
    s.radialG=100;assert(!filter.sample(s,18500,true));
    s.radialG=-8;s.x=2000;assert(!filter.sample(s,21000,true));
    s.x=0;assert(filter.sample(s,23500,true));
    s.radialG=-380;assert(filter.sample(s,26000,false)); // Comparison can turn impact rejection off.

    ChannelRecovery left,right;FakeDriver l,r;
    for(uint32_t now=1000;now<2600000;now+=1000){left.step(l,now,250);right.step(r,now,250);}
    assert(left.ready()&&right.ready());assert(l.zeros==2500&&l.directions==10&&l.modes==10);
    l.status=Io::BUSY;auto submissions=r.throttles;
    left.step(l,2600000,300);right.step(r,2600000,300);
    assert(left.ready() && left.submitted==250 && right.submitted==300 && r.throttles==submissions+1);
    l.status=Io::OK;assert(left.step(l,2601000,300) && left.submitted==300);
    l.status=Io::ERROR;
    for(uint32_t now=2602000;now<2650000;now+=1000){left.step(l,now,400);right.step(r,now,400);}
    assert(l.restarts==1 && !left.ready() && right.ready() && right.submitted==400);
    l.status=Io::OK;left.step(l,2800000,400);assert(left.ready());assert(left.step(l,2801000,400));
    assert(l.zeros==2500 && l.modes==10); // RMT recovery does not reset a live ESC's mode or feed it startup zeros.
    ChannelRecovery unavailable;FakeDriver absent;absent.status=Io::ERROR;
    for(uint32_t now=1;now<1000000;now+=1000)unavailable.step(absent,now,250);
    absent.status=Io::OK;for(uint32_t now=1000000;now<3800000;now+=1000)unavailable.step(absent,now,250);
    assert(unavailable.ready());

    Tuning t;assert(validTuning(t));auto record=tuningRecord(t);Tuning decoded;
    assert(decodeTuning(record,decoded));record.offset=NAN;record.checksum=tuningChecksum(record);assert(!decodeTuning(record,decoded));
    record=tuningRecord(t);record.checksum^=1;assert(!decodeTuning(record,decoded));
    t.phaseOffset=30;t.leadMs=5;assert(fabsf(modulationPhase(0,625,false,t)-48.75f)<.001f);
    assert(fabsf(modulationPhase(0,625,true,t)-348.75f)<.001f);
    t.phaseOffset=t.leadMs=0;
    auto strong=mixMelty(250,88,false,180,t);assert(strong.delta==699 && strong.left==949 && strong.right==-449);
    t.profile=Profile::V5_REFERENCE;auto legacy=mixMelty(250,88,false,180,t);assert(legacy.delta==760 && legacy.left==999 && legacy.right==-510);
    for(unsigned profile=0;profile<4;++profile)for(int throttle=11;throttle<=100;++throttle)
        for(int stick=0;stick<=100;++stick)for(unsigned reverse=0;reverse<2;++reverse)for(unsigned angle=0;angle<360;angle+=3) {
            t.profile=Profile(profile);auto m=mixMelty(throttle*10,stick,reverse,float(angle),t);
            assert(abs(m.left)<=999 && abs(m.right)<=999);
            if(profile==2){assert(m.left+m.right==2*m.base);assert(reverse?(m.left<=0&&m.right<=0):(m.left>=0&&m.right>=0));}
            if(stick>=40 && stick<=60)assert(m.delta==0);
        }
    SpinRamp ramp;assert(ramp.step(25,false,1000)==0);assert(ramp.step(25,false,11000)==25);
    assert(ramp.step(25,false,111000)==150);assert(ramp.step(25,false,151000)==250);
    assert(ramp.step(25,true,161000)==225);assert(ramp.step(0,false,162000)==0);
    assert(markerWindow(359,0,625) && !markerWindow(30,0,625));
    Events<3> events;for(unsigned n=0;n<5;++n)events.add(n,EventKind::ARM,n);assert(events.count==3 && events.at(0).value==2);
    Timing timing;timing.step(1,false);timing.step(100000,true);assert(timing.maxGap==0);
    timing.step(100166,true);timing.step(104000,true);assert(timing.maxGap==3834 && timing.late==1);
    std::cout<<"PASS: five-second loss/rearm/rollover; continuing sensor prediction; shock rejection/recovery; independent channel retry; 8.7 million mixer cases; phase tuning, ramp, LEDs and settings.\n";
}
