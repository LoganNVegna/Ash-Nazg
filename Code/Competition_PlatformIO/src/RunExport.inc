// Frozen capture export. Fixed lengths, byte resume and small row pages share
// the same formatter/checksum. Nonblocking socket writes bound idle stalls.
static unsigned exportRows(bool test) {return test ? 800 : rowsRetained;}
static BaseRow exportRow(bool test,unsigned i) {
    if(!test)return baseRows[(writeIndex+1500-rowsRetained+i)%1500];
    BaseRow r{};r.us=i*10000;r.sensorUs=r.us;r.polls=i*10;r.fresh=i;r.x=i%10;r.y=-int(i%20);
    r.z=1;r.ch3=15;r.translation=50;r.wantedL=r.wantedR=r.left=r.right=150;
    r.bodyRpm=600;r.phase=i%360;r.trim=1;r.status=15;r.outputTicks=i*60;return r;
}
struct ExportMeta {size_t bytes=0;String sha;bool valid=true;};
static ExportMeta exportMeta(bool test,unsigned start,unsigned count) {
    ExportMeta result;char line[640];unsigned char digest[32];mbedtls_sha256_context hash;
    mbedtls_sha256_init(&hash);mbedtls_sha256_starts_ret(&hash,0);
    size_t n=strlen(baselineCsvHeader);result.bytes+=n;
    mbedtls_sha256_update_ret(&hash,(const unsigned char*)baselineCsvHeader,n);
    for(unsigned i=start;i<start+count;++i) {
        int length=formatBaselineRow(line,sizeof(line),exportRow(test,i));
        if(length<0 || size_t(length)>=sizeof(line)){result.valid=false;break;}
        result.bytes+=length;mbedtls_sha256_update_ret(&hash,(const unsigned char*)line,length);
        if((i%25)==0){feedBaselineWatchdog();delay(1);}
    }
    mbedtls_sha256_finish_ret(&hash,digest);mbedtls_sha256_free(&hash);
    char hex[65];for(unsigned i=0;i<32;++i)snprintf(hex+i*2,3,"%02x",digest[i]);hex[64]=0;
    result.sha=hex;return result;
}
static bool exportWrite(WiFiClient& client,const char* data,size_t length) {
    size_t offset=0;uint32_t progress=millis();
    while(offset<length) {
        feedBaselineWatchdog();int written=::send(client.fd(),data+offset,length-offset,MSG_DONTWAIT);
        if(written>0){offset+=written;progress=millis();}
        else if(written==0 || (errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR))return false;
        if(uint32_t(millis()-progress)>=2000)return false;
        delay(1);
    }
    return true;
}
static bool exportNumber(const String& value,size_t& number) {
    if(!value.length())return false;number=0;
    for(unsigned i=0;i<value.length();++i){
        if(value[i]<'0' || value[i]>'9' || number>10000000)return false;
        number=number*10+value[i]-'0';
    }
    return true;
}
static void downloadExport(bool test) {
    if(!maintenance){baselineServer.send(409,"text/plain","Capture is running.");return;}
    unsigned totalRows=exportRows(test),start=0,count=totalRows;
    bool paged=baselineServer.hasArg("start") || baselineServer.hasArg("count");
    if(paged) {
        size_t a=0,b=25;
        if((baselineServer.hasArg("start") && !exportNumber(baselineServer.arg("start"),a)) ||
           (baselineServer.hasArg("count") && !exportNumber(baselineServer.arg("count"),b)) || a>totalRows || b==0 || b>50) {
            baselineServer.send(400,"text/plain","Use start >= 0 and count 1..50.");return;
        }
        start=a;count=min(unsigned(b),totalRows-start);
    }
    ExportMeta meta=exportMeta(test,start,count);
    if(!meta.valid){baselineServer.send(500,"text/plain","CSV formatting failed.");return;}
    String etag="\""+String(exportId,HEX)+(test?"-check-":"-run-")+String(rowsTotal)+"\"";
    size_t first=0,last=meta.bytes-1;int code=200;
    String range=baselineServer.header("Range"),ifRange=baselineServer.header("If-Range");
    if(!paged && range.length() && (!ifRange.length() || ifRange==etag)) {
        bool ok=range.startsWith("bytes=") && range.indexOf(',')<0;
        int dash=range.indexOf('-',6);size_t parsed=0;
        ok=ok && dash>6 && exportNumber(range.substring(6,dash),first);
        if(ok && range.substring(dash+1).length()) {
            ok=exportNumber(range.substring(dash+1),parsed);if(ok)last=min(parsed,last);
        }
        if(!ok || first>last || first>=meta.bytes) {
            baselineServer.sendHeader("Content-Range","bytes */"+String((unsigned)meta.bytes));
            baselineServer.send(416,"text/plain","Invalid byte range.");return;
        }
        code=206;baselineServer.sendHeader("Content-Range","bytes "+String((unsigned)first)+"-"+String((unsigned)last)+"/"+String((unsigned)meta.bytes));
    }
    baselineServer.sendHeader("Content-Disposition",test?"attachment; filename=AshNazg-download-check.csv":"attachment; filename=AshNazg-competition.csv");
    baselineServer.sendHeader("Accept-Ranges",paged?"none":"bytes");
    baselineServer.sendHeader("Cache-Control","no-store");baselineServer.sendHeader("ETag",etag);
    baselineServer.sendHeader("X-CSV-SHA256",meta.sha);
    baselineServer.setContentLength(last-first+1);baselineServer.send(code,"text/csv","");
    WiFiClient client=baselineServer.client();char packet[1024],line[640];size_t filled=0,position=0;
    auto append=[&](const char* data,size_t length) {
        size_t offset=0,n=baselineSlice(position,length,first,last,offset);position+=length;data+=offset;
        while(n) {
            size_t piece=min(n,sizeof(packet)-filled);memcpy(packet+filled,data,piece);filled+=piece;data+=piece;n-=piece;
            if(filled==sizeof(packet)){if(!exportWrite(client,packet,filled))return false;filled=0;}
        }
        return true;
    };
    bool ok=append(baselineCsvHeader,strlen(baselineCsvHeader));
    for(unsigned i=start;ok && i<start+count && position<=last;++i) {
        int n=formatBaselineRow(line,sizeof(line),exportRow(test,i));ok=n>=0 && size_t(n)<sizeof(line);
        if(ok)ok=append(line,n);
        if((i%25)==0){feedBaselineWatchdog();delay(1);}
    }
    if(ok && filled)ok=exportWrite(client,packet,filled);
    if(!ok)++downloadFailures;
    client.stop(); // Explicit EOF; partial writes leave capture intact for retry.
}
static void downloadBaseline(){downloadExport(false);}
static void exportManifest() {
    bool test=baselineServer.arg("test")=="1";auto meta=exportMeta(test,0,exportRows(test));
    if(!meta.valid){baselineServer.send(500,"text/plain","CSV formatting failed.");return;}
    String json=String("{\"build\":\"")+BUILD_NAME+"\",\"capture_id\":\""+String(exportId,HEX)+"-"+String(rowsTotal)+
        "\",\"test\":"+(test?"true":"false")+",\"rows\":"+String(exportRows(test))+
        ",\"bytes\":"+String((unsigned)meta.bytes)+",\"sha256\":\""+meta.sha+"\"}";
    baselineServer.sendHeader("Cache-Control","no-store");baselineServer.send(200,"application/json",json);
}
