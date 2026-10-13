#pragma once
#include <cstdint>
class IPAddress {
public:
    IPAddress(uint8_t a = 0, uint8_t b = 0, uint8_t c = 0, uint8_t d = 0) : bytes_{a,b,c,d} {}
    uint8_t operator[](int index) const { return bytes_[index]; }
private:
    uint8_t bytes_[4];
};
