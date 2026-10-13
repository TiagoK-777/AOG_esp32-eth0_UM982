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
void poll() {
    uint32_t now = millis();
    SteerStatus status = state::steerSnapshot();
    GuidanceCommand command = state::guidanceSnapshot();
    if (!status.available || status.steerSwitch || !commandFresh(now, command) ||
        !(command.guidanceStatus & 1) || command.gpsSpeed < 1.5f || command.gpsSpeed > 25.0f ||
        (status.cycles && now - status.completedMs > CONTROL_STALE_MS) || pending::maintenanceActive())
        motor::stop();
}
}
