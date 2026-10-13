#pragma once
#include <cstdint>
namespace network {
bool setup();
void poll();
struct Stats { uint32_t invalidPackets, sendFailures; };
Stats stats();
}
