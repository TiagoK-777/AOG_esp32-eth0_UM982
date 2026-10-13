#pragma once
#include <cstdint>
struct GuidanceCommand {
    float gpsSpeed = 0;
    float steerAngleSetPoint = 0;
    uint8_t guidanceStatus = 0;
    uint8_t tram = 0, relay = 0, relayHi = 0;
    uint32_t receivedMs = 0;
    uint32_t sequence = 0;
    uint32_t transitionSequence = 0;
    bool valid = false;
};
namespace state {
GuidanceCommand guidanceSnapshot();
void publishGuidance(GuidanceCommand value);
}
