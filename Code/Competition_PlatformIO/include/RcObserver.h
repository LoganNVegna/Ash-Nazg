#pragma once
#include <stdint.h>
class RcObserver {
    uint8_t buf[64]{};
    unsigned used = 0;
    uint32_t previousByteUs = 0;
public:
    uint16_t channels[16]{};
    uint32_t lastRcUs = 0, lastLinkUs = 0, frames = 0, linkFrames = 0, crcErrors = 0, lengthErrors = 0;
    uint8_t lq = 255;
    bool haveChannels = false, linkLost = false;
    void resetParser(){used=0;previousByteUs=0;}
    static uint8_t crc(const uint8_t* p, unsigned n) {
        uint8_t c = 0;
        while (n--) { c ^= *p++; for (int i = 0; i < 8; ++i) c = (c & 0x80) ? uint8_t((c << 1) ^ 0xd5) : uint8_t(c << 1); }
        return c;
    }
    bool feed(uint8_t b, uint32_t now) {
        if (used && uint32_t(now - previousByteUs) > 3000) used = 0;
        previousByteUs = now;
        if (!used) { if (b != 0xc8 && b != 0x00) return false; buf[used++] = b; return false; }
        if (used == 1 && (b < 2 || b > 62)) { ++lengthErrors; used = 0; return false; }
        buf[used++] = b;
        if (used < unsigned(buf[1]) + 2) return false;
        const unsigned length = buf[1];
        used = 0;
        if (crc(buf + 2, length - 1) != buf[length + 1]) { ++crcErrors; return false; }
        if (buf[2] == 0x14 && length == 12) {
            lq = buf[5]; linkLost = (lq == 0); lastLinkUs=now; ++linkFrames; return false;
        }
        if (buf[2] != 0x16 || length != 24) return false;
        for (unsigned ch = 0; ch < 16; ++ch) {
            uint16_t v = 0;
            for (unsigned bit = 0; bit < 11; ++bit) {
                const unsigned pos = ch * 11 + bit;
                v |= uint16_t((buf[3 + pos / 8] >> (pos % 8)) & 1) << bit;
            }
            channels[ch] = v;
        }
        haveChannels = true; lastRcUs = now; ++frames; return true;
    }
};


