#include "state/guidance_state.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
namespace {
GuidanceCommand command;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
}
namespace state {
GuidanceCommand guidanceSnapshot() {
    portENTER_CRITICAL(&mux);
    GuidanceCommand copy = command;
    portEXIT_CRITICAL(&mux);
    return copy;
}
void publishGuidance(GuidanceCommand value) {
    portENTER_CRITICAL(&mux);
    value.sequence = command.sequence + 1;
    value.transitionSequence = command.transitionSequence +
        (value.guidanceStatus != command.guidanceStatus);
    command = value;
    portEXIT_CRITICAL(&mux);
}
}
