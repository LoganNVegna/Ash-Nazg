#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>

namespace ash {
constexpr float SENSOR_G_PER_COUNT=0.195f, SENSOR_RADIUS_M=0.018f;
constexpr uint32_t RC_TIMEOUT_US=100000, LINK_TIMEOUT_US=1000000;
constexpr uint32_t SENSOR_TIMEOUT_US=100000, MAIN_TIMEOUT_US=50000;
inline float clamp(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
inline uint32_t age(uint32_t now,uint32_t before){return now-before;}
inline float wrap(float v){v=fmodf(v,360.0f);return v<0?v+360:v;}
inline float rpmFromG(float g){return sqrtf(fabsf(g)*9.80665f/SENSOR_RADIUS_M)*9.549296586f;}
inline int channelPercent(uint16_t raw) {
    // Preserve the original library's rcToUs conversion and Arduino map truncation.
    int us=int(raw*0.62477120195241f+881.0f);
    return int(clamp(float((us-1000)*100/1000),0,100));
}
struct Rc {
    int16_t ch[8]={50,50,0,50,100,0,50,50};
    uint32_t time=0,linkTime=0,frames=0,crcErrors=0;
    uint8_t lq=0;
    bool seen=false,linkSeen=false;
};
inline bool neutral(const Rc& r){return r.ch[2]<=10 && r.ch[0]>=45 && r.ch[0]<=55 && r.ch[1]>=45 && r.ch[1]<=55;}
inline bool rcFresh(const Rc& r,uint32_t now){return r.seen && age(now,r.time)<=RC_TIMEOUT_US;}
inline bool linkHealthy(const Rc& r,uint32_t now){return r.linkSeen && r.lq>0 && age(now,r.linkTime)<=LINK_TIMEOUT_US;}
struct Sensor {
    uint32_t time=0,polls=0,fresh=0,overruns=0;
    int16_t x=0,y=0,z=0;
    float biasCounts=0,radialG=0,filteredG=0,rpm=0;
    uint8_t status=0,who=0,ctrl1=0,ctrl4=0;
    bool calibrated=false,rail=false,failed=false;
};
struct Estimator {
    float filtered=0;uint32_t previous=0;bool have=false;
    void sample(float g,uint32_t now) {
        if(!have){filtered=g;have=true;}
        else {float dt=float(age(now,previous))*1e-6f;filtered+=(g-filtered)*(1.0f-expf(-dt/0.020f));}
        previous=now;
    }
};
enum class Stop:uint8_t {BOOT,KILL,RC_STALE,LINK_LOST,SENSOR_STALE,MAIN_STALE,OUTPUT_ERROR,SENSOR_RAIL,INIT_FAILED,OTA};
inline const char* stopName(Stop s) {
    const char* n[]={"boot","CH5_high","RC_stale","receiver_link_lost","sensor_fresh_sample_stale","main_loop_stale","DShot_API_error","sensor_saturated","initialization_failed","OTA"};
    return n[unsigned(s)];
}
struct SafetyInputs {Rc rc;Sensor sensor;uint32_t heartbeat=0;bool ready=false,fatal=false,updating=false,exporting=false,saving=false;};
inline Stop fault(const SafetyInputs& i,uint32_t now) {
    if(i.updating)return Stop::OTA;
    if(i.fatal || i.sensor.failed)return Stop::INIT_FAILED;
    if(i.rc.ch[4]>50)return Stop::KILL;
    if(!rcFresh(i.rc,now))return Stop::RC_STALE;
    if(!linkHealthy(i.rc,now))return Stop::LINK_LOST;
    if(age(now,i.heartbeat)>MAIN_TIMEOUT_US)return Stop::MAIN_STALE;
    if(!i.sensor.calibrated || age(now,i.sensor.time)>SENSOR_TIMEOUT_US)return Stop::SENSOR_STALE;
    if(i.sensor.rail)return Stop::SENSOR_RAIL;
    return Stop::BOOT;
}
struct ArmGate {
    bool armed=false,seenHigh=false;Stop stop=Stop::BOOT;
    uint32_t stoppedAt=0,generation=0;bool everStopped=false;
    void disarm(Stop why,uint32_t now) {
        if(armed){stoppedAt=now;everStopped=true;stop=why;}
        armed=false;seenHigh=false;
    }
    bool step(const SafetyInputs& i,uint32_t now) {
        if(!i.ready || i.fatal || i.updating || i.exporting || i.saving || i.sensor.failed) {
            if(!armed && !everStopped && (i.fatal || i.sensor.failed))stop=Stop::INIT_FAILED;
            disarm(i.updating?Stop::OTA:Stop::INIT_FAILED,now);return false;
        }
        if(armed){Stop why=fault(i,now);if(why!=Stop::BOOT){disarm(why,now);return false;}return true;}
        bool healthy=rcFresh(i.rc,now) && linkHealthy(i.rc,now) && i.sensor.calibrated && !i.sensor.rail &&
            age(now,i.sensor.time)<=SENSOR_TIMEOUT_US && age(now,i.heartbeat)<=MAIN_TIMEOUT_US;
        if(!healthy || !neutral(i.rc)){seenHigh=false;return false;}
        // Require control and link evidence after a stop, not a held pre-stop snapshot.
        if(everStopped && (int32_t(i.rc.time-stoppedAt)<=0 || int32_t(i.rc.linkTime-stoppedAt)<=0))return false;
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
inline Mix melty(int throttle,int translation,bool reverse,float rpm,float phase) {
    Mix m;if(throttle<=10)return m;
    float requested=clamp(float(throttle*10),0,999);
    float b=fminf(requested,floorf(0.4f*rpm)+200.0f);
    float u=clamp((fabsf(float(translation-50))-10.0f)/40.0f,0,1);
    b=fminf(b,999.0f/(1.0f+u));
    int base=int(b),peak=int(u*base);
    // Swapping the faster wheel on the opposite half-turn produces the same
    // world-direction push. Use both halves without increasing peak commands.
    float wave=-cosf(wrap(phase)*0.01745329252f);
    bool swap=(translation<50)^reverse;
    int d=int(wave*peak)*(swap?-1:1);int signedBase=reverse?-base:base;
    m.left=int16_t(signedBase+d);m.right=int16_t(signedBase-d);m.base=signedBase;m.delta=d;m.strength=u;
    m.mode=u>0?Drive::TRANSLATE:Drive::SPIN;return m;
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
