#pragma once
#include "config/settings.h"
int16_t calcSteeringPID(float error, const Storage& settings, const Setup& config);
