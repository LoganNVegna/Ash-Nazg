// The motor owner alone calls this: preserve V5 pins, frames, delays and mode commands.
static bool startupEscs() {
    if(escR.install(GPIO_NUM_18,RMT_CHANNEL_3,600000UL,3)!=ESP_OK)return false;
    rightInstalled=true;
    if(escL.install(GPIO_NUM_17,RMT_CHANNEL_2,600000UL,3)!=ESP_OK)return false;
    leftInstalled=true;
    for(int i=0;i<2500;++i) {
        if(startupInterrupted())return false;
        if(escR.writeData(0,true)!=ESP_OK)return false;
        delayMicroseconds(400);
        if(escL.writeData(0,true)!=ESP_OK)return false;
        delayMicroseconds(400);
    }
    if(startupInterrupted())return false;
    if(escR.setReversed(false)!=ESP_OK)return false;
    if(escR.set3DMode(true)!=ESP_OK)return false;
    delayMicroseconds(200);
    if(escL.setReversed(false)!=ESP_OK)return false;
    if(escL.set3DMode(true)!=ESP_OK)return false;
    return true;
}
