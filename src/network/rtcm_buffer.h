#pragma once
#include <cstddef>
#include <cstdint>
namespace rtcm {
bool enqueue(const uint8_t* data, size_t length);
size_t read(uint8_t* data, size_t capacity);
size_t available();
uint32_t droppedPackets();
}
