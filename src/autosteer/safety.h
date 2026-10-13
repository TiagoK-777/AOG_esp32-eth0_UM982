#pragma once
#include "state/guidance_state.h"
namespace safety {
bool commandFresh(uint32_t now, const GuidanceCommand& command);
void poll();
}
