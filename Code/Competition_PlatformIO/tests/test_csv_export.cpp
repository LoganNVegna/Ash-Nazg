#include "RunCsv.h"
#include <cassert>
#include <cstring>
#include <string>
#include <cerrno>
#include <algorithm>
#include <iostream>
#include <limits>

static uint32_t nowMs=0;
static unsigned feeds=0,calls=0;
static int mode=0;
static std::string sent;
static uint32_t millis(){return nowMs;}
static void delay(unsigned n){nowMs+=n;}
static void feedBaselineWatchdog(){++feeds;}
struct WiFiClient {int fd(){return 7;}};
static const int MSG_DONTWAIT=1;
static int send(int fd,const char* data,size_t length,int flags) {
    assert(fd==7 && flags==MSG_DONTWAIT);++calls;
    if(mode==1 || (mode==0 && calls%3==1)){errno=EAGAIN;return -1;}
    if(mode==2){errno=ECONNRESET;return -1;}
    if(mode==3)return 0;
    size_t n=std::min(length,size_t(7));sent.append(data,n);return int(n);
}
// Generated from the actual firmware function, with no behavioral substitutions.
#include "generated/export_write.inc"

int main() {
    BaseRow row{};row.sensorUs=uint32_t(-123);row.y=-2048;row.trim=1;
    char line[640];int n=formatBaselineRow(line,sizeof(line),row);
    assert(n>0 && size_t(n)<sizeof(line));
    assert(std::string(line).find("0,-123,")==0);
    assert(std::count(line,line+n,',')==31 && line[n-1]=='\n');
    char tiny[8];assert(formatBaselineRow(tiny,sizeof(tiny),row)>=int(sizeof(tiny)));
    row.us=row.polls=row.fresh=row.overruns=row.rcAge=row.outputGap=row.errors=row.outputTicks=UINT32_MAX;
    row.bodyRpm=INT32_MIN;row.x=row.y=row.z=row.ch3=row.translation=row.ch5=row.wantedL=row.wantedR=row.left=row.right=row.phase=INT16_MIN;
    row.legacyG=row.trim=std::numeric_limits<float>::max();row.status=row.stop=255;
    n=formatBaselineRow(line,sizeof(line),row);assert(n>0 && size_t(n)<sizeof(line));
    const std::string body=std::string(baselineCsvHeader)+line;
    // Every possible start/end range, split across deliberately irregular fragments.
    for(size_t first=0;first<body.size();++first)for(size_t last=first;last<body.size();++last) {
        std::string result;
        for(size_t p=0;p<body.size();) {
            size_t len=std::min(size_t(1+p%31),body.size()-p),offset=0;
            size_t take=baselineSlice(p,len,first,last,offset);
            if(take)result.append(body,p+offset,take);p+=len;
        }
        assert(result==body.substr(first,last-first+1));
    }
    size_t offset=99;assert(baselineSlice(0,0,0,0,offset)==0 && offset==0);
    WiFiClient client;
    assert(exportWrite(client,body.data(),body.size()) && sent==body && feeds==calls);
    mode=1;nowMs=0;feeds=calls=0;
    assert(!exportWrite(client,"abc",3) && nowMs==2000 && feeds>=2000);
    mode=2;assert(!exportWrite(client,"abc",3));
    mode=3;assert(!exportWrite(client,"abc",3));
    mode=1;nowMs=UINT32_MAX-1000;
    assert(!exportWrite(client,"abc",3) && nowMs==999); // Timer rollover.
    std::cout<<"PASS: CSV formatting, all byte slices, partial writes, stalls, disconnects and timer rollover.\n";
}
