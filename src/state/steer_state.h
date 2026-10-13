#pragma once
#include <cstdint>
struct SteerStatus {
    bool available = false;
    bool engaged = false;
    float steerAngleActual = 0;
    float sensorReading = 0;
    int16_t helloSteerPosition = 0;
    uint8_t pwmDisplay = 0;
    uint8_t switchByte = 2;
    uint8_t steerSwitch = 1;
    uint32_t completedMs = 0;
    uint32_t cycles = 0;
    uint32_t maxCycleUs = 0;
};
namespace state {
SteerStatus steerSnapshot();
void publishSteer(const SteerStatus& value);
}
