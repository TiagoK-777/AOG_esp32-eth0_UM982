#pragma once
#include "state/imu_state.h"
#include "config/settings.h"
namespace imu {
void setup();
void poll(ImuState& sample, const Setup& config);
}
