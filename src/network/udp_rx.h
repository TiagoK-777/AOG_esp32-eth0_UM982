#pragma once
#include <cstdint>
namespace network {
bool setup();
void poll();
struct Stats {
    uint32_t invalidPackets = 0, sendFailures = 0;
    uint32_t gpsSent = 0, gpsFailed = 0, gpsGapUs = 0;
};
Stats stats();
}
