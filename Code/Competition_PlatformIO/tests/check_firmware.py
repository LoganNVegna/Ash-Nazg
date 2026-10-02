from pathlib import Path
import re,struct,csv,json,hashlib,statistics,math
p=Path(__file__).resolve().parents[1]
for name in ('DShotESC.cpp','DShotESC.h'):
    assert (p/'src'/name).read_bytes()==(p/'reference'/name).read_bytes(),name+' differs from V5'
generated=p/'tests/generated';generated.mkdir(exist_ok=True)
driver=(p/'src/DShotESC.cpp').read_text()
driver=re.sub(r'#define DSHOT_ERROR_CHECK\(x\).*?\}\)',
 '#define DSHOT_ERROR_CHECK(x) do { esp_err_t e=(x); if(e!=ESP_OK)return e; } while(0)',driver,count=1,flags=re.S)
(generated/'driver.cpp').write_text(driver)
test=r'''
#include "DShotESC.h"
#include <stdio.h>
std::vector<MockFrame> trace;
static DShotESC escL,escR;
static bool rightInstalled=false,leftInstalled=false;
constexpr gpio_num_t GPIO_NUM_18=18,GPIO_NUM_17=17;
static bool interruptStartup=false;
static bool startupInterrupted(){return interruptStartup;}
#include "../../src/EscStartup.inc"
static bool validStartup(){
 if(trace.size()!=5040)return false;
 for(int i=0;i<5000;++i)if(trace[i].channel!=(i%2?2:3) || trace[i].packet!=0)return false;
 for(int i=5000;i<5040;++i){int channel=i<5020?3:2;unsigned command=(i%20)<10?20:10;
  if(trace[i].channel!=channel || (trace[i].packet>>5)!=command || !(trace[i].packet&16))return false;}
 return true;
}
int main(){
 assert(startupEscs());assert(validStartup());
 const int commands[]={0,1,110,200,999,1000,-1,-110,-999,-1000};
 const unsigned payloads[]={0,1048,1157,1247,2047,2047,49,158,1047,1047};
 for(unsigned i=0;i<10;++i){escL.sendThrottle3D(commands[i]);assert((trace.back().packet>>5)==payloads[i]);}
 trace.clear();interruptStartup=true;assert(!startupEscs() && trace.empty());
 puts("PASS: actual competition startup, original stop/settings packets, encoder/pulse timings and startup OTA interruption.");
}
'''
(generated/'test_driver.cpp').write_text(test)
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
print(f'PASS: byte-identical driver, ESP32-S3 image ({len(image)} bytes), OTA partitions/config.')
