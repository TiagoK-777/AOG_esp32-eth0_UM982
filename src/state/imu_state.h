#pragma once
#include <cstdint>
struct ImuState {
    bool useBNO08x = false;
    double yaw = 0, pitch = 0, roll = 0;
    double gyroZ = 0, correctionHeading = 0;
    uint32_t lastRvcTime = 0, rvcPacketCount = 0;
};
namespace state {
ImuState imuSnapshot();
void publishImu(const ImuState& value);
}
