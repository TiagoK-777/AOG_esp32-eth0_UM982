#pragma once
#include "Arduino.h"
class TwoWire {
public:
    uint8_t error = 0;
    uint8_t count = 2;
    uint8_t bytes[2] = {0x35, 0x2A}; // WAS centrado: 13610 / 2 = 6805.
    uint8_t index = 0;
    void begin(int, int) {}
    void setTimeOut(uint16_t) {}
    void beginTransmission(uint8_t) {}
    size_t write(uint8_t) { return 1; }
    uint8_t endTransmission() { return error; }
    uint8_t requestFrom(uint8_t, uint8_t) { index = 0; return count; }
    int read() { return index < count ? bytes[index++] : -1; }
};
inline TwoWire Wire;
