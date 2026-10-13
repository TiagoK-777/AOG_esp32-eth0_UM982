#pragma once
#include "config/settings.h"
namespace persist {
bool load(ConfigSnapshot& values);
bool save(const ConfigSnapshot& values, uint8_t mask);
}
