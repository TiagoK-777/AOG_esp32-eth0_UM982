#pragma once
#include "diagnostics/timing.h"
namespace autosteer {
bool setup(bool motorReady);
void poll();
diagnostics::Timing takeCycleTiming();
diagnostics::Timing takeAdcTiming();
}
