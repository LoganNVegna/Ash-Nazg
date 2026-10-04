#pragma once
#include "ControlCore.h"

namespace ash {
enum class Quality:uint8_t {MEASURED,PREDICTED,UNAVAILABLE};
inline const char* qualityName(Quality q){return q==Quality::MEASURED?"measured":q==Quality::PREDICTED?"predicted":"unavailable";}

// Reject isolated shocks before the EMA. Genuine sustained changes are accepted
// after three consistent samples, and angular-rate corrections are slew limited.
struct RadialFilter {
    Estimator ema;float candidate=0,lastRpm=0;unsigned candidates=0;
    uint32_t last=0;bool have=false;
    bool sample(const Sensor& s,uint32_t now,bool robust) {
        float g=s.radialG;
        if(s.failed || s.rail || !isfinite(g))return false;
        if(robust) {
            if(g>2.0f)return false; // Installed Y axis points opposite centripetal acceleration.
            float cross=hypotf(float(s.x),float(s.z))*SENSOR_G_PER_COUNT;
            if(cross>fmaxf(30.0f,2*fabsf(g)))return false;
            if(have && fabsf(g-ema.filtered)>fmaxf(12.0f,0.5f*fabsf(ema.filtered))) {
                if(candidates && fabsf(g-candidate)<fmaxf(4.0f,0.2f*fabsf(g)))++candidates;
                else {candidate=g;candidates=1;}
                if(candidates<3)return false;
            }
        }
        candidates=0;ema.sample(g,now);
        float measured=fabsf(ema.filtered)<0.75f?0:rpmFromG(ema.filtered);
        if(have && robust) {
            float allowed=20000*clamp(age(now,last)*1e-6f,0,0.02f);
            lastRpm+=clamp(measured-lastRpm,-allowed,allowed);
        } else lastRpm=measured;
        have=true;last=now;return true;
    }
};

struct RateTracker {
    float rpm=0,learned=0,acceptedCommand=0,lastAcceptedRpm=0;
    uint32_t previous=0,lastSample=0;bool have=false,haveMeasurement=false;
    Quality quality=Quality::UNAVAILABLE;
    float step(const Sensor& s,uint32_t now,float spin,float fallback) {
        float dt=have?fminf(age(now,previous)*1e-6f,0.02f):0;
        previous=now;have=true;
        bool usable=s.accepted && (s.calibrated || s.biasKnown) && !s.failed && !s.rail && !s.impact && isfinite(s.rpm) && age(now,s.acceptedTime)<100000;
        if(usable && (!haveMeasurement || s.acceptedTime!=lastSample)) {
            if(!haveMeasurement)rpm=s.rpm;
            // Learn only from reasonably stable commanded spin, not a startup/reversal.
            if(fabsf(spin)>100 && s.rpm>450 && fabsf(fabsf(spin)-acceptedCommand)<2) {
                float slope=clamp(s.rpm/fabsf(spin),0.1f,20);
                learned=learned>0?learned+0.02f*(slope-learned):slope;
            }
            acceptedCommand=fabsf(spin);lastAcceptedRpm=s.rpm;
            lastSample=s.acceptedTime;haveMeasurement=true;
        }
        if(usable)rpm+=clamp(s.rpm-rpm,-20000*dt,20000*dt);
        quality=usable?Quality::MEASURED:Quality::PREDICTED;
        // Brief outages hold angular rate. Longer outages use the last measured
        // command/rate relation without changing requested motor output or mode.
        if(!haveMeasurement || age(now,lastSample)>300000) {
            float slope=learned>0?learned:fallback;
            float target=haveMeasurement?fmaxf(0,lastAcceptedRpm+slope*(fabsf(spin)-acceptedCommand)):fabsf(spin)*slope;
            rpm+=(target-rpm)*(1-expf(-dt/1.0f));
            if(!haveMeasurement && rpm<1)quality=Quality::UNAVAILABLE;
        }
        if(!isfinite(rpm))rpm=haveMeasurement?lastAcceptedRpm:0;
        rpm=clamp(rpm,0,10000);return rpm;
    }
};

enum class Io:uint8_t {OK,BUSY,ERROR};
enum class ChannelStage:uint8_t {INSTALL,ZEROS,DIRECTION,MODE,READY};
// IO exposes install/recover/stop/direction/mode/throttle, each one bounded
// submission. Lifecycle is per channel; no operation touches its sibling.
struct ChannelRecovery {
    ChannelStage stage=ChannelStage::INSTALL;
    uint32_t next=0,errors=0,busy=0,recoveries=0,lastSuccess=0,firstError=0;
    unsigned count=0,consecutive=0;int16_t submitted=0;uint32_t submittedAt=0;
    bool everReady=false,unavailable=true;
    bool ready()const{return stage==ChannelStage::READY;}
    template<class Driver> bool step(Driver& io,uint32_t now,int16_t wanted) {
        if(next && int32_t(now-next)<0)return false;
        next=0;Io result;
        if(stage==ChannelStage::INSTALL) {
            result=io.install(everReady);
            if(result==Io::OK){stage=everReady?ChannelStage::READY:ChannelStage::ZEROS;count=0;consecutive=0;unavailable=false;}
            else {++errors;unavailable=true;next=now+100000;}
            return false;
        }
        switch(stage) {
            case ChannelStage::ZEROS:result=io.stop();break;
            case ChannelStage::DIRECTION:result=io.direction();break;
            case ChannelStage::MODE:result=io.mode();break;
            default:result=io.throttle(wanted);break;
        }
        if(result!=Io::OK) {
            if(result==Io::BUSY)++busy;else ++errors;
            if(!consecutive)firstError=now;++consecutive;
            // A late frame isn't a drive fault. Reinstall only after a persistent
            // problem; the other channel continues receiving its own commands.
            if(consecutive>=32 && age(now,firstError)>=20000) {
                io.recover();stage=ChannelStage::INSTALL;count=0;next=now+1000;
                unavailable=true;++recoveries;
            }
            return false;
        }
        consecutive=0;lastSuccess=now;
        if(stage==ChannelStage::READY){submitted=wanted;submittedAt=now;return true;}
        submitted=0;submittedAt=now;
        if(stage==ChannelStage::ZEROS){next=now+800;if(++count>=2500){stage=ChannelStage::DIRECTION;count=0;}}
        else if(stage==ChannelStage::DIRECTION){if(++count>=10){stage=ChannelStage::MODE;count=0;}next=now+200;}
        else if(stage==ChannelStage::MODE){if(++count>=10){stage=ChannelStage::READY;everReady=true;}next=now+200;}
        return false;
    }
};

enum class EventKind:uint8_t {ARM,STOP,SENSOR_REJECT,SENSOR_RECOVER,LEFT_RECOVER,RIGHT_RECOVER,OUTPUT_GAP};
struct Event {uint32_t time;EventKind kind;int32_t value;};
template<unsigned N=64> struct Events {
    Event entries[N]{};unsigned count=0,index=0;
    void add(uint32_t time,EventKind kind,int32_t value=0){entries[index]={time,kind,value};index=(index+1)%N;if(count<N)++count;}
    Event at(unsigned n)const{return entries[(index+N-count+n)%N];}
};
struct Timing {
    uint32_t previous=0,maxGap=0,late=0;bool have=false,wasDriving=false;
    uint32_t step(uint32_t now,bool driving) {
        uint32_t gap=have?age(now,previous):0;previous=now;have=true;
        if(driving && wasDriving){if(gap>maxGap)maxGap=gap;if(gap>1000)++late;}
        wasDriving=driving;
        return gap;
    }
};
struct TuningRecord {uint32_t magic=0x415a5432;uint32_t profile=0,flags=4;float gain=1,offset=0,lead=0,model=4;uint32_t checksum=0;};
inline uint32_t tuningChecksum(const TuningRecord& r){const auto* b=reinterpret_cast<const uint8_t*>(&r);uint32_t h=2166136261u;for(unsigned i=0;i<sizeof(r)-4;++i)h=(h^b[i])*16777619u;return h;}
inline TuningRecord tuningRecord(const Tuning& t){TuningRecord r;r.profile=unsigned(t.profile);r.flags=(t.diagnosticLeds?1:0)|(t.capture?2:0)|(t.robustFilter?4:0);r.gain=t.gain;r.offset=t.phaseOffset;r.lead=t.leadMs;r.model=t.rpmPerCommand;r.checksum=tuningChecksum(r);return r;}
inline bool decodeTuning(const TuningRecord& r,Tuning& t) {
    if(r.magic!=0x415a5432 || r.profile>3 || r.flags>7 || r.checksum!=tuningChecksum(r))return false;
    Tuning candidate;candidate.profile=Profile(r.profile);candidate.gain=r.gain;candidate.phaseOffset=r.offset;candidate.leadMs=r.lead;candidate.rpmPerCommand=r.model;
    candidate.diagnosticLeds=r.flags&1;candidate.capture=r.flags&2;candidate.robustFilter=r.flags&4;
    if(!validTuning(candidate))return false;t=candidate;return true;
}
} // namespace ash
