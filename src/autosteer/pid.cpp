/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/


#include "autosteer/pid.h"
#include "config/defines.h"
#include <cmath>

int16_t calcSteeringPID(float error, const Storage& settings, const Setup& config) {
    int16_t drive = static_cast<int16_t>(settings.Kp * error);
    float magnitude = std::fabs(error);
    float slope = static_cast<float>(settings.highPWM - settings.lowPWM) / LOW_HIGH_DEGREES;
    int16_t limit = magnitude < LOW_HIGH_DEGREES
        ? static_cast<int16_t>(magnitude * slope + settings.lowPWM) : settings.highPWM;
    if (drive < 0) drive -= settings.minPWM;
    else if (drive > 0) drive += settings.minPWM;
    if (drive > limit) drive = limit;
    if (drive < -limit) drive = -limit;
    if (config.MotorDriveDirection) drive *= -1;
    if (config.IsDanfoss) {
        if (drive > 250) drive = 250;
        if (drive < -250) drive = -250;
        drive = (drive >> 2) + 128;
    }
    return drive;
}
