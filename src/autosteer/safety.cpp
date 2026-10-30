#include <Arduino.h>
#include "autosteer/safety.h"
#include "config/defines.h"
#include "state/steer_state.h"
#include "state/pending.h"
#include "hardware/motor_pwm.h"
namespace safety {
bool commandFresh(uint32_t now, const GuidanceCommand& command) {
    return command.valid && now - command.receivedMs < COMMAND_TIMEOUT_MS;
}
bool pwmSpeedAllowed(const GuidanceCommand& command) {
    return command.gpsSpeed >= MIN_STEER_PWM_SPEED_KMH && command.gpsSpeed <= MAX_STEER_SPEED_KMH;
}
void poll() {
    uint32_t now = millis();
    SteerStatus status = state::steerSnapshot();
    GuidanceCommand command = state::guidanceSnapshot();
    if (!status.available || !status.engaged || status.steerSwitch || !commandFresh(now, command) ||
        !(command.guidanceStatus & 1) || !pwmSpeedAllowed(command) ||
        (status.cycles && now - status.completedMs > CONTROL_STALE_MS) || pending::maintenanceActive())
        motor::stop(motor::StopSource::Safety);
}
}
