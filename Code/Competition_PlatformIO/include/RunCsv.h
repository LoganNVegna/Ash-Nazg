#pragma once
#include <stdint.h>
#include <stdio.h>
struct BaseRow {
    uint32_t us,sensorUs,polls,fresh,overruns,rcAge,outputGap,errors,outputTicks,freshAge;
    int16_t x,y,z,ch3,translation,ch5,wantedL,wantedR,left,right,base,delta;
    float phase,bodyRpm,phaseRate,legacyG,trim;
    uint16_t computeUs;
    uint8_t status,stop,mode,lq;
};
static const char baselineCsvHeader[]="time_us,sensor_sample_time_us,sensor_polls,sensor_ready_count,sensor_overruns,rc_age_us,max_output_gap_us,output_errors,raw_x,raw_y,raw_z,throttle_pct,translation_pct,ch5_pct,wanted_left,wanted_right,submitted_left,submitted_right,phase_deg,physical_rpm,filtered_radial_g,trim_period,status,stop_code,output_iterations,fresh_sample_age_us,phase_rate_rpm,mix_base,mix_delta,drive_mode,link_quality,output_compute_us\n";
inline int formatBaselineRow(char* line,size_t capacity,const BaseRow& r) {
    return snprintf(line,capacity,"%lu,%ld,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.5f,%.6f,%u,%u,%lu,%lu,%.3f,%d,%d,%u,%u,%u\n",
        (unsigned long)r.us,(long)int32_t(r.sensorUs),(unsigned long)r.polls,(unsigned long)r.fresh,(unsigned long)r.overruns,
        (unsigned long)r.rcAge,(unsigned long)r.outputGap,(unsigned long)r.errors,r.x,r.y,r.z,r.ch3,r.translation,r.ch5,
        r.wantedL,r.wantedR,r.left,r.right,r.phase,r.bodyRpm,r.legacyG,r.trim,r.status,r.stop,(unsigned long)r.outputTicks,
        (unsigned long)r.freshAge,r.phaseRate,r.base,r.delta,r.mode,r.lq,r.computeUs);
}
inline size_t baselineSlice(size_t position,size_t length,size_t first,size_t last,size_t& offset) {
    offset=0;if(!length || position>last || position+length<=first)return 0;
    offset=first>position?first-position:0;size_t end=position+length-1;if(end>last)end=last;
    return end-(position+offset)+1;
}
