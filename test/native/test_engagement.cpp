#include <Arduino.h>
#include <Wire.h>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include "autosteer/autosteer_task.h"
#include "autosteer/safety.h"
#include "config/pinout.h"
#include "hardware/motor_pwm.h"
#include "network/pgn_protocol.h"
#include "state/pending.h"
#include "state/steer_state.h"

const UserSettings userSettings{};
namespace {
int mode = 0;
void tick() {
    fakeMillis += 25;
    autosteer::poll();
    safety::poll();
}
void send(uint16_t speed, bool active) {
    // PGN 254 do AOG: velocidade x10, status, setpoint = 1 grau.
    uint8_t packet[]{0x80,0x81,0x7F,254,8,static_cast<uint8_t>(speed),
        static_cast<uint8_t>(speed >> 8),static_cast<uint8_t>(active),100,0,0,0,0,0};
    pgn::checksum(packet, sizeof(packet));
    GuidanceCommand command;
    assert(pgn::decodeGuidance(packet, sizeof(packet), command));
    command.receivedMs = fakeMillis;
    state::publishGuidance(command);
    tick();
}
void stopped() {
    assert(state::steerSnapshot().pwmDisplay == 0);
    assert(fakePwm[PWM1_LPWM] == 0 && fakePwm[PWM2_RPWM] == 0);
}
void driving() {
    const auto status = state::steerSnapshot();
    assert(status.engaged && status.pwmDisplay > 0);
    assert(fakePwm[PWM1_LPWM] > 0 || fakePwm[PWM2_RPWM] > 0);
}
void arm() {
    if (mode) { fakePins[STEERSW_PIN] = LOW; tick(); }
    send(0, true);
    assert(state::steerSnapshot().steerSwitch == 0);
    assert(state::steerSnapshot().engaged);
    assert((state::steerSnapshot().switchByte & 2) == 0);
    stopped();
    if (mode == 2) { fakePins[STEERSW_PIN] = HIGH; tick(); }
}
}
int main(int argc, char** argv) {
    assert(argc == 3);
    mode = std::atoi(argv[1]); // none, switch, button
    const int driver = std::atoi(argv[2]); // Cytron, IBT2, Danfoss
    ConfigSnapshot values;
    values.steerConfig.SteerSwitch = mode == 1;
    values.steerConfig.SteerButton = mode == 2;
    values.steerConfig.CytronDriver = driver != 1;
    values.steerConfig.IsDanfoss = driver == 2;
    config::publish(values);
    fakePins[STEERSW_PIN] = HIGH;
    assert(motor::setup() && autosteer::setup(true));
    send(0, false);
    stopped();
    arm();

    // Fica engajado parado e abaixo de 1,5 km/h, sem PWM, inclusive Danfoss.
    send(0, true); stopped();
    assert(state::steerSnapshot().engaged);
    send(14, true); stopped();
    assert(state::steerSnapshot().engaged);
    send(15, true); driving();

    // Free Drive oficial usa speed=80 (8 km/h), status=1, mesmo parado.
    send(80, true); driving();
    // Ao sair do teste, o primeiro PGN com velocidade zero já corta o motor.
    send(0, true); stopped();
    assert(state::steerSnapshot().engaged);
    send(80, true); driving();

    // O supervisor corta mesmo antes da próxima passagem do controle.
    GuidanceCommand command = state::guidanceSnapshot();
    command.gpsSpeed = 0; command.receivedMs = fakeMillis;
    state::publishGuidance(command);
    safety::poll();
    assert(fakePwm[PWM1_LPWM] == 0 && fakePwm[PWM2_RPWM] == 0);
    tick(); stopped();
    send(80, true); driving();
    command = state::guidanceSnapshot();
    command.guidanceStatus = 0;
    state::publishGuidance(command);
    safety::poll();
    assert(fakePwm[PWM1_LPWM] == 0 && fakePwm[PWM2_RPWM] == 0);
    tick();
    assert(!state::steerSnapshot().engaged);
    stopped();

    // Desengajamento pela entrada física; None acompanha a GUI.
    send(80, true);
    if (mode == 1) { fakePins[STEERSW_PIN] = HIGH; tick(); }
    else if (mode == 2) { fakePins[STEERSW_PIN] = LOW; tick(); }
    else send(0, false);
    assert(state::steerSnapshot().steerSwitch == 1);
    assert(!state::steerSnapshot().engaged);
    stopped();

    if (mode == 2) { fakePins[STEERSW_PIN] = HIGH; tick(); }
    if (mode) { fakePins[STEERSW_PIN] = LOW; tick(); }
    send(80, false); stopped();
    send(80, true); driving();

    // Free Drive continua sujeito a timeout, ADC, limite superior e persistência.
    fakeMillis += 2500;
    safety::poll();
    assert(fakePwm[PWM1_LPWM] == 0 && fakePwm[PWM2_RPWM] == 0);
    tick(); stopped();
    assert(!state::steerSnapshot().engaged);
    send(80, true); driving();
    Wire.error = 2;
    send(80, true); stopped();
    assert(!state::steerSnapshot().engaged);
    Wire.error = 0;
    if (mode) { fakePins[STEERSW_PIN] = HIGH; tick(); }
    send(0, false); arm();
    send(80, true); driving();
    send(251, true); stopped();
    assert(!state::steerSnapshot().engaged);
    if (mode) { fakePins[STEERSW_PIN] = HIGH; tick(); }
    send(0, false); arm();
    send(80, true); driving();
    pending::stageSettings(values.steerSettings);
    tick(); stopped();
    assert(pending::maintenanceActive() && !state::steerSnapshot().engaged);
    pending::Update update;
    assert(pending::takePersistence(update));
    pending::finishPersistence();
    send(80, true); stopped();
    assert(!state::steerSnapshot().engaged);
    std::cout << "Engajamento modo=" << mode << " driver=" << driver << " OK\n";
}
