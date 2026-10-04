#pragma once
#include "../include/ControlCore.h"
namespace ash {
// Historical bounded 1.2 mixer, retained for comparison only.
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
}
