/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
Forked from https://github.com/AgHardware/Boards/blob/main/TeensyModules/AIO%20Standard%20v4/Firmware/Autosteer_gps_teensy_v4/Autosteer_gps_teensy_v4.ino
*/


#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config/settings.h"
#include "config/defines.h"
#include "persist/preferences.h"
#include "autosteer/autosteer_task.h"
#include "hardware/motor_pwm.h"
#include "gps/gps.h"
#include "network/ethernet.h"
#include "network/udp_rx.h"
#include "state/pending.h"
#include "tasks.h"

/************************* User Settings *************************/
const UserSettings userSettings = []() constexpr {
    UserSettings settings;
    settings.udpPassthrough = false;
    settings.makeOGI = false;
    settings.baseLineCheck = false; // true: sincroniza GGA/HPR e habilita fusão com IMU.
    settings.headingcorr = 0; // 0: baseline frente/trás (pitch); 900: lado a lado (roll).
    settings.filterRoll = false;
    settings.filterHeading = false;
    settings.sendUSB = false;
    settings.isIBT2CurrentSensor = false;
    return settings;
}();
/************************* End User Settings **********************/

namespace { bool running = false; }
void setup() {
    Serial.begin(115200);
    Serial.println("Start setup");
    bool motorReady = motor::setup();
    ConfigSnapshot values;
    bool settingsReady = persist::load(values);
    if (!settingsReady) Serial.println("NVS load FAILED - steering disabled");
    config::publish(values);
    bool available = autosteer::setup(motorReady && settingsReady);
    gps::setup();
    if (!ethernet::setup(available) || !network::setup()) {
        Serial.println("Network initialization FAILED - motor disabled");
        motor::stop();
        return;
    }
    running = tasks::create();
    if (!running) { motor::stop(); Serial.println("Task initialization FAILED - motor disabled"); }
    Serial.println("End setup, waiting for GPS...");
}
void loop() {
    if (running) {
        pending::Update update;
        if (pending::takePersistence(update)) {
            // A task de controle já confirmou o corte antes de entregar este pedido.
            if (persist::save(update.values, update.mask)) {
                Serial.println("Configuration saved to NVS");
                if (update.mask & pending::Network) ESP.restart();
                pending::finishPersistence();
            } else {
                Serial.println("NVS save FAILED - steering remains disabled");
            }
        }
        static uint32_t lastDiagnostic = 0;
        if (millis() - lastDiagnostic >= 5000) {
            lastDiagnostic = millis();
            tasks::diagnostics();
        }
    }
    vTaskDelay(pdMS_TO_TICKS(MAINTENANCE_PERIOD_MS));
}
