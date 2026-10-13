#include "state/pending.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
namespace {
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
pending::Update staged, persistence;
bool maintenance = false;
}
namespace pending {
void stageSettings(const Storage& value) {
    portENTER_CRITICAL(&mux);
    staged.values.steerSettings = value;
    staged.mask |= Settings;
    portEXIT_CRITICAL(&mux);
}
void stageConfig(const Setup& value) {
    portENTER_CRITICAL(&mux);
    staged.values.steerConfig = value;
    staged.mask |= Config;
    portEXIT_CRITICAL(&mux);
}
void stageNetwork(const ConfigIP& value) {
    portENTER_CRITICAL(&mux);
    staged.values.networkAddress = value;
    staged.mask |= Network;
    portEXIT_CRITICAL(&mux);
}
bool take(Update& update) {
    portENTER_CRITICAL(&mux);
    bool ready = !maintenance && staged.mask;
    if (ready) { update = staged; staged.mask = 0; }
    portEXIT_CRITICAL(&mux);
    return ready;
}
void requestPersistence(const Update& update) {
    portENTER_CRITICAL(&mux);
    persistence = update;
    maintenance = true;
    portEXIT_CRITICAL(&mux);
}
bool takePersistence(Update& update) {
    portENTER_CRITICAL(&mux);
    bool ready = persistence.mask != 0;
    if (ready) { update = persistence; persistence.mask = 0; }
    portEXIT_CRITICAL(&mux);
    return ready;
}
bool maintenanceActive() {
    portENTER_CRITICAL(&mux);
    bool active = maintenance;
    portEXIT_CRITICAL(&mux);
    return active;
}
void finishPersistence() {
    portENTER_CRITICAL(&mux);
    maintenance = false;
    portEXIT_CRITICAL(&mux);
}
}
