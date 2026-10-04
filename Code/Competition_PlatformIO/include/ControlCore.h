#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>

namespace ash {
constexpr float SENSOR_G_PER_COUNT=0.195f, SENSOR_RADIUS_M=0.018f;
constexpr uint32_t RC_TIMEOUT_US=5000000, ARM_FRAME_US=250000;
inline float clamp(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
inline uint32_t age(uint32_t now,uint32_t before){return now-before;}
inline float wrap(float v){v=fmodf(v,360.0f);return v<0?v+360:v;}
inline bool markerWindow(float phase,float center,float rpm){float d=fabsf(wrap(phase-center+180)-180);return d<=fminf(35,fmaxf(7.5f,fabsf(rpm)*0.000006f*800));}
inline float rpmFromG(float g){return sqrtf(fabsf(g)*9.80665f/SENSOR_RADIUS_M)*9.549296586f;}
inline int channelPercent(uint16_t raw) {
    // Preserve the original library's rcToUs conversion and Arduino map truncation.
    int us=int(raw*0.62477120195241f+881.0f);
    return int(clamp(float((us-1000)*100/1000),0,100));
}
struct Rc {
    int16_t ch[8]={50,50,0,50,100,0,50,50};
    uint32_t time=0,linkTime=0,frames=0,crcErrors=0,liveTime=0,sequence=0;
    uint8_t lq=0;
    bool seen=false,linkSeen=false,liveSeen=false,reportedLoss=false;
};
inline bool neutral(const Rc& r){return r.ch[2]<=10 && r.ch[0]>=45 && r.ch[0]<=55 && r.ch[1]>=45 && r.ch[1]<=55;}
inline bool rcFresh(const Rc& r,uint32_t now){return r.liveSeen && age(now,r.liveTime)<RC_TIMEOUT_US;}
inline bool liveFrame(const Rc& r,uint32_t now){return r.liveSeen && !r.reportedLoss && age(now,r.liveTime)<=ARM_FRAME_US;}
struct Sensor {
    uint32_t time=0,polls=0,fresh=0,overruns=0,acceptedTime=0,accepted=0,rejected=0,recoveries=0,rails=0;
    int16_t x=0,y=0,z=0;
    float biasCounts=0,radialG=0,filteredG=0,rpm=0;
    uint8_t status=0,who=0,ctrl1=0,ctrl4=0;
    bool calibrated=false,biasKnown=false,rail=false,failed=false,impact=false;
};
struct Estimator {
    float filtered=0;uint32_t previous=0;bool have=false;
    void sample(float g,uint32_t now) {
        if(!have){filtered=g;have=true;}
        else {float dt=float(age(now,previous))*1e-6f;filtered+=(g-filtered)*(1.0f-expf(-dt/0.020f));}
        previous=now;
    }
};
// Legacy CSV numbers remain stable; 1.4 emits only BOOT/KILL/RC_STALE/OTA.
enum class Stop:uint8_t {BOOT,KILL,RC_STALE,LINK_LOST,SENSOR_STALE,MAIN_STALE,OUTPUT_ERROR,SENSOR_RAIL,INIT_FAILED,OTA};
inline const char* stopName(Stop s) {
    const char* n[]={"boot","CH5_high","RC_stale","receiver_link_lost","sensor_fresh_sample_stale","main_loop_stale","DShot_API_error","sensor_saturated","initialization_failed","OTA"};
    return n[unsigned(s)];
}
// Component health is deliberately absent: it cannot disarm the operator.
struct SafetyInputs {Rc rc;bool ready=false,updating=false,exporting=false,saving=false;};
struct ArmGate {
    bool armed=false,seenHigh=false;Stop stop=Stop::BOOT;
    uint32_t stoppedAt=0,generation=0,lastSequence=0;bool everStopped=false;
    void disarm(Stop why,uint32_t now) {
        if(armed){stoppedAt=now;everStopped=true;stop=why;}
        armed=false;seenHigh=false;
    }
    bool step(const SafetyInputs& i,uint32_t now) {
        bool newFrame=i.rc.sequence!=lastSequence;lastSequence=i.rc.sequence;
        if(i.updating){disarm(Stop::OTA,now);return false;}
        if(armed) {
            if(!rcFresh(i.rc,now)){disarm(Stop::RC_STALE,now);return false;}
            if(liveFrame(i.rc,now) && i.rc.ch[4]>50){disarm(Stop::KILL,now);return false;}
            return true;
        }
        // Flash/export work may delay arming, never stop a running robot.
        if(!i.ready){seenHigh=false;return false;}
        if(i.exporting || i.saving)return false;
        if(!newFrame || !liveFrame(i.rc,now))return false;
        if(everStopped && int32_t(i.rc.liveTime-stoppedAt)<=0)return false;
        // Retain neutral boot arming; recovery needs only a fresh CH5 cycle.
        if(!everStopped && !neutral(i.rc)){seenHigh=false;return false;}
        if(i.rc.ch[4]>50)seenHigh=true;
        else if(seenHigh){armed=true;seenHigh=false;stop=Stop::BOOT;++generation;return true;}
        return false;
    }
};
struct Phase {
    float degrees=0;uint32_t previous=0;bool have=false;
    void reset(uint32_t now){degrees=0;previous=now;have=true;}
    void step(uint32_t now,float rateRpm){if(!have)reset(now);degrees=wrap(degrees+float(age(now,previous))*rateRpm*0.000006f);previous=now;}
};
enum class Drive:uint8_t {STOP,SPIN,TRANSLATE,TANK,UNSTICK};
struct CaptureWindow {
    uint32_t lastMotion=0;bool sawMotion=false;
    bool keep(Drive mode,uint32_t now,bool force=false) {
        if(mode!=Drive::STOP){lastMotion=now;sawMotion=true;}
        // Keep a short coast-down tail, then preserve driving evidence through idle waits.
        return force || (sawMotion && age(now,lastMotion)<=1000000);
    }
};
struct Mix {int16_t left=0,right=0,base=0,delta=0;float strength=0;Drive mode=Drive::STOP;};
enum class Profile:uint8_t {STRONG_HALF,STRONG_FULL,BOUNDED_FULL,V5_REFERENCE};
inline const char* profileName(Profile p){const char* names[]={"strong_half","strong_full","bounded_full","v5_reference"};return names[unsigned(p)];}
struct Tuning {
    Profile profile=Profile::STRONG_HALF;
    float gain=1,phaseOffset=0,leadMs=0,rpmPerCommand=4;
    bool diagnosticLeds=false,capture=false,robustFilter=true;
};
inline bool validTuning(const Tuning& t) {
    return unsigned(t.profile)<=3 && isfinite(t.gain) && t.gain>=0 && t.gain<=1.5f &&
        (t.profile!=Profile::BOUNDED_FULL || t.gain<=1) &&
        isfinite(t.phaseOffset) && t.phaseOffset>=-180 && t.phaseOffset<=180 &&
        isfinite(t.leadMs) && t.leadMs>=-20 && t.leadMs<=20 &&
        isfinite(t.rpmPerCommand) && t.rpmPerCommand>=0.1f && t.rpmPerCommand<=20;
}
inline float modulationPhase(float phase,float rpm,bool reverse,const Tuning& t) {
    // Phase is spin-relative, as in V5. A physical offset mirrors in reverse;
    // a time lead advances along the current spin direction in either case.
    return wrap(phase+t.phaseOffset*(reverse?-1:1)+clamp(rpm*0.006f*t.leadMs,-150,150));
}
// Time-based spin ramp never depends on sensor quality or an RPM-derived cap.
struct SpinRamp {
    float command=0;uint32_t previous=0;bool have=false;
    float step(int throttle,bool reverse,uint32_t now) {
        float target=throttle>10?clamp(float(throttle*10),0,999)*(reverse?-1:1):0;
        float dt=have?fminf(age(now,previous)*1e-6f,0.05f):0;
        previous=now;have=true;
        if(target==0){command=0;return 0;}
        // Operator reductions take effect immediately; ramp increases/reversal.
        if(command*target>=0 && fabsf(target)<fabsf(command))command=target;
        else command+=clamp(target-command,-2500*dt,2500*dt);
        return command;
    }
};
inline Mix mixMelty(float spin,int translation,bool reverse,float phase,const Tuning& t) {
    Mix m;float b=clamp(fabsf(spin),0,999);
    float u=clamp((fabsf(float(translation-50))-10.0f)/40.0f,0,1);
    float peak;
    if(t.profile==Profile::BOUNDED_FULL){b=fminf(b,999.0f/(1.0f+u));peak=u*b;}
    else if(t.profile==Profile::V5_REFERENCE)peak=u>0?20.0f*fabsf(float(translation-50)):0;
    else peak=u*999;
    peak*=t.profile==Profile::BOUNDED_FULL?fminf(t.gain,1.0f):t.gain;
    float wave=-cosf(wrap(phase)*0.01745329252f);
    if(t.profile==Profile::STRONG_HALF || t.profile==Profile::V5_REFERENCE)wave=fmaxf(0,wave);
    int d=int(wave*peak)*(((translation<50)^reverse)?-1:1);
    int base=int(b)*(reverse?-1:1);
    m.left=int16_t(clamp(float(base+d),-999,999));m.right=int16_t(clamp(float(base-d),-999,999));
    m.base=base;m.delta=d;m.strength=u;m.mode=u>0?Drive::TRANSLATE:Drive::SPIN;return m;
}
inline int mapInt(int v,int inLo,int inHi,int outLo,int outHi){return (v-inLo)*(outHi-outLo)/(inHi-inLo)+outLo;}
inline Mix ground(const Rc& r) {
    Mix m;int c1=r.ch[0],c2=r.ch[1];
    if(r.ch[5]>50 && (c2>55 || c2<45)) {m.left=mapInt(c2,0,100,-120,120);m.right=mapInt(c2,0,100,120,-120);m.mode=Drive::UNSTICK;return m;}
    if(c1>=45 && c1<=55)c1=50;if(c2>=45 && c2<=55)c2=50;
    if(c1==50 && c2==50)return m;
    int t=mapInt(c1,0,100,-260,260),turn=mapInt(c2,0,100,260,-260);
    int l=int(clamp(float(t+turn),-260,260)),rr=int(clamp(float(t-turn),-260,260));
    if(l>0 && l<60)l=60;if(l<0 && l>-60)l=-60;
    if(rr>0 && rr<60)rr=60;if(rr<0 && rr>-60)rr=-60;
    m.left=l;m.right=rr;m.mode=Drive::TANK;return m;
}
struct Trim {
    float period=1;bool excursion=false,dirty=false;uint32_t changedAt=0;bool display=false;
    bool adjust(int channel,uint32_t now) {
        if(channel>40 && channel<60){excursion=false;return false;}
        if(excursion || (channel>=30 && channel<=70))return false;
        excursion=true;float next=clamp(period+(channel<30?-0.004f:0.004f),0.9f,1.1f);
        changedAt=now;display=true;if(next==period)return false;period=next;dirty=true;return true;
    }
    bool visible(uint32_t now)const{return display && (excursion || age(now,changedAt)<1500000);}
    float level()const{return clamp((period-0.9f)*45.0f,0,9);}
};
inline uint8_t trimPixel(float level,unsigned index,uint8_t brightness=85) {
    return uint8_t(lroundf(clamp(level-float(index),0,1)*brightness));
}
struct TrimRecord {uint32_t magic;float period;uint32_t checksum;};
inline uint32_t trimChecksum(const TrimRecord& r){const uint8_t* b=(const uint8_t*)&r;uint32_t h=2166136261u;for(unsigned i=0;i<8;++i)h=(h^b[i])*16777619u;return h;}
inline TrimRecord trimRecord(float period){TrimRecord r{0x415a5431u,period,0};r.checksum=trimChecksum(r);return r;}
inline bool trimValid(const TrimRecord& r){return r.magic==0x415a5431u && isfinite(r.period) && r.period>=0.9f && r.period<=1.1f && r.checksum==trimChecksum(r);}
} // namespace ash
