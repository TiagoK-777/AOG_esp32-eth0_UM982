#include "network/rtcm_buffer.h"
#include "config/defines.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
namespace {
uint8_t buffer[RTCM_BUFFER_SIZE];
size_t head = 0, tail = 0, count = 0;
uint32_t dropped = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
}
namespace rtcm {
bool enqueue(const uint8_t* data, size_t length) {
    portENTER_CRITICAL(&mux);
    if (length > RTCM_BUFFER_SIZE - count) {
        ++dropped;
        portEXIT_CRITICAL(&mux);
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        buffer[head] = data[i];
        head = (head + 1) % RTCM_BUFFER_SIZE;
    }
    count += length;
    portEXIT_CRITICAL(&mux);
    return true;
}
size_t read(uint8_t* data, size_t capacity) {
    portENTER_CRITICAL(&mux);
    size_t length = capacity < count ? capacity : count;
    for (size_t i = 0; i < length; ++i) {
        data[i] = buffer[tail];
        tail = (tail + 1) % RTCM_BUFFER_SIZE;
    }
    count -= length;
    portEXIT_CRITICAL(&mux);
    return length;
}
size_t available() {
    portENTER_CRITICAL(&mux);
    size_t value = count;
    portEXIT_CRITICAL(&mux);
    return value;
}
uint32_t droppedPackets() {
    portENTER_CRITICAL(&mux);
    uint32_t value = dropped;
    portEXIT_CRITICAL(&mux);
    return value;
}
}
