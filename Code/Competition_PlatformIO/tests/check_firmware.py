from pathlib import Path
import re,struct,csv,json,hashlib,statistics,math
p=Path(__file__).resolve().parents[1]
def block(text, prefix):
    start=text.index(prefix);begin=text.index('{',start);end=begin+1;depth=1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]
generated=p/'tests/generated';generated.mkdir(exist_ok=True)
driver=(p/'src/DShotESC.cpp').read_text()
driver=re.sub(r'#define DSHOT_ERROR_CHECK\(x\).*?\}\)',
 '#define DSHOT_ERROR_CHECK(x) do { esp_err_t e=(x); if(e!=ESP_OK)return e; } while(0)',driver,count=1,flags=re.S)
(generated/'driver.cpp').write_text(driver)
runtime=(p/'src/Runtime.inc').read_text()
esc_port=block(runtime,'struct EscPort')+';\n'
test=r'''
#include "DShotESC.h"
#include "RecoveryCore.h"
#include "runtime_mocks.h"
#include <stdio.h>
std::vector<MockFrame> trace;
''' + esc_port + r'''
int main(){
 DShotESC escL,escR;EscPort l(escL,17,RMT_CHANNEL_2),r(escR,18,RMT_CHANNEL_3);
 ash::ChannelRecovery left,right;
 for(uint32_t now=1000;!(left.ready()&&right.ready());now+=1000){assert(now<3000000);right.step(r,now,0);left.step(l,now+25,0);}
 assert(trace.size()==5040);
 for(unsigned channel=2;channel<=3;++channel){unsigned n=0;
  for(auto frame:trace)if(frame.channel==int(channel)){
   if(n<2500)assert(frame.packet==0);
   else {assert((frame.packet>>5)==(n<2510?20:10));assert(frame.packet&16);}++n;
  }assert(n==2520);
 }
 const int commands[]={0,1,110,200,999,1000,-1,-110,-999,-1000};
 const unsigned payloads[]={0,1048,1157,1247,2047,2047,49,158,1047,1047};
 for(unsigned i=0;i<10;++i){escL.sendThrottle3D(commands[i]);assert((trace.back().packet>>5)==payloads[i]);}
 assert(escL.sendStartupCommand(DSHOT_CMD::SAVE_SETTINGS)==ESP_ERR_INVALID_ARG);
 mockFault[2]=ESP_ERR_TIMEOUT;auto before=trace.size();assert(l.throttle(200)==ash::Io::BUSY && trace.size()==before);
 assert(r.throttle(200)==ash::Io::OK && trace.back().channel==3);mockFault[2]=ESP_OK;
 puts("PASS: actual channel startup/transport, original 2500 stops and 10+10 settings packets, signed encoder and pulse timing, busy response and independent sibling output.");
}
'''
(generated/'test_driver.cpp').write_text(test)
main=(p/'src/main.cpp').read_text();maintenance=(p/'src/Maintenance.inc').read_text()
begin=main.index('struct Output');end=main.index('static Shared state;',begin)
(generated/'runtime_state.inc').write_text(main[begin:end]+'static Shared state;\n'+block(main,'static Shared snapshot')+'\n')
helpers='\n'.join(block(main,sig) for sig in ('static bool zeroAcknowledged','static bool waitForZero'))
helpers+='\n'+'\n'.join(block(maintenance,sig) for sig in ('static void saveSettings','static void networkTick','static void networkTask','static void ensureTasks'))
callbacks='\n'.join(block(maintenance,prefix)+');' for prefix in ('ArduinoOTA.onStart(', 'ArduinoOTA.onProgress(', 'ArduinoOTA.onError('))
(generated/'runtime_maintenance.inc').write_text(helpers+'\nstatic void installCallbacks(){\n'+callbacks+'\n}\n')
assert 'vTaskDelete' not in runtime+maintenance and 'portMAX_DELAY' not in runtime
assert 'ESP.restart' not in runtime+maintenance
export=(p/'src/RunExport.inc').read_text()
assert 'BUILD_NAME' in export and 'AshNazg competition 1.0' not in export, 'Export identity must use the firmware build name'
match=re.search(r'static bool exportWrite\([^;]*?\)\s*\{',export);assert match
pos=export.index('{',match.start());end=pos+1;depth=1
while depth:depth+=(export[end]=='{')-(export[end]=='}');end+=1
(generated/'export_write.inc').write_text(export[match.start():end])
build=p/'.pio/build/upload_ota';image=(build/'firmware.bin').read_bytes()
assert image[0]==0xe9 and struct.unpack_from('<H',image,12)[0]==9
expected_name=re.search(r'BUILD_NAME\[\]="([^"]+)"', (p/'src/main.cpp').read_text()).group(1)
assert len(image)<0x1e0000 and expected_name.encode() in image
assert (build/'partitions.bin').read_bytes()==(p/'reference/ota-partitions.bin').read_bytes()
config=(p/'platformio.ini').read_text();assert 'upload_protocol = espota' in config and '--auth=admin' in config
capture=p.parent/'AshNazg-V5-baseline/captures/run-bb35393e-1400'
if capture.exists():
    data=(capture/'AshNazg-V5-baseline.csv').read_bytes();manifest=json.loads((capture/'manifest.json').read_text(encoding='utf-8-sig'))
    assert hashlib.sha256(data).hexdigest()==manifest['sha256']
    rows=list(csv.DictReader(data.decode().splitlines()))
    before=[float(r['v5_body_rpm']) for r in rows if 5740000<=int(r['time_us'])<6212207]
    corrected=statistics.median(before)*math.sqrt(.098/(400/2047))
    assert 620<corrected<625
    print(f'PASS: saved pre-translation estimate corrects to {corrected:.2f} RPM.')
print(f'PASS: ESP32-S3 image ({len(image)} bytes), OTA partitions/config; generated tests use actual production transport, lifecycle and maintenance code.')
