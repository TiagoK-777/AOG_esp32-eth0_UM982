#pragma once
#include "config/settings.h"
namespace motor {
bool setup();
uint32_t epoch();
uint8_t drive(int16_t pwm, const Setup& config, uint32_t expectedEpoch);
void stop();
}
