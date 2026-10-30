#pragma once
#include "config/settings.h"
namespace motor {
enum class StopSource : uint8_t { Control, Safety, Other };
constexpr uint8_t TIMING_COUNT = 5;
struct OperationTiming {
    uint32_t calls = 0, totalUs = 0, waitUs = 0, bodyUs = 0;
};
struct Timing { OperationTiming operations[TIMING_COUNT]; };
Timing takeTiming();
bool setup();
uint32_t epoch();
uint8_t drive(int16_t pwm, const Setup& config, uint32_t expectedEpoch);
void stop(StopSource source = StopSource::Other);
}
