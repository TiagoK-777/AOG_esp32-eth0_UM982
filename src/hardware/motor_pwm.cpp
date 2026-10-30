/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/


#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "hardware/motor_pwm.h"
#include "config/pinout.h"
#include "config/defines.h"
namespace {
StaticSemaphore_t lockStorage;
SemaphoreHandle_t lock = nullptr;
uint32_t generation = 0;
bool cutApplied = false;
portMUX_TYPE timingMux = portMUX_INITIALIZER_UNLOCKED;
motor::Timing timing{};
void recordTiming(uint8_t operation, uint32_t startUs, uint32_t acquiredUs, uint32_t finishedUs) {
    uint32_t wait = acquiredUs - startUs, body = finishedUs - acquiredUs;
    uint32_t total = finishedUs - startUs;
    portENTER_CRITICAL(&timingMux);
    auto& value = timing.operations[operation];
    ++value.calls;
    if (wait > value.waitUs) value.waitUs = wait;
    if (body > value.bodyUs) value.bodyUs = body;
    if (total > value.totalUs) value.totalUs = total;
    portEXIT_CRITICAL(&timingMux);
}
void cut() {
    digitalWrite(DIR1_RL_ENABLE, LOW);
    // Repetir ledcWrite no ESP32 pode esperar pelo período PWM anterior.
    // Só omitir quando ambas as escritas anteriores tiveram sucesso.
    if (!cutApplied) {
        bool first = ledcWrite(PWM1_LPWM, 0);
        bool second = ledcWrite(PWM2_RPWM, 0);
        cutApplied = first && second;
    }
}
}
namespace motor {
Timing takeTiming() {
    portENTER_CRITICAL(&timingMux);
    Timing copy = timing;
    timing = {};
    portEXIT_CRITICAL(&timingMux);
    return copy;
}
bool setup() {
    cutApplied = false;
    pinMode(DIR1_RL_ENABLE, OUTPUT);
    digitalWrite(DIR1_RL_ENABLE, LOW);
    uint32_t frequency = PWM_Frequency == 1 ? 10000 : PWM_Frequency == 2 ? 15000 : 490;
    bool first = ledcAttach(PWM1_LPWM, frequency, 8);
    bool second = ledcAttach(PWM2_RPWM, frequency, 8);
    cut();
    lock = xSemaphoreCreateMutexStatic(&lockStorage);
    return first && second && lock;
}
uint32_t epoch() {
    if (!lock) return 0;
    uint32_t startedUs = micros();
    xSemaphoreTake(lock, portMAX_DELAY);
    uint32_t acquiredUs = micros();
    uint32_t value = generation;
    uint32_t finishedUs = micros();
    xSemaphoreGive(lock);
    recordTiming(4, startedUs, acquiredUs, finishedUs);
    return value;
}
void stop(StopSource source) {
    if (!lock) return;
    uint32_t startedUs = micros();
    xSemaphoreTake(lock, portMAX_DELAY);
    uint32_t acquiredUs = micros();
    ++generation;
    cut();
    uint32_t finishedUs = micros();
    xSemaphoreGive(lock);
    recordTiming(static_cast<uint8_t>(source), startedUs, acquiredUs, finishedUs);
}
uint8_t drive(int16_t pwm, const Setup& config, uint32_t expectedEpoch) {
    if (!lock) return 0;
    uint32_t startedUs = micros();
    xSemaphoreTake(lock, portMAX_DELAY);
    uint32_t acquiredUs = micros();
    // Um corte concorrente invalida o cálculo que já estava em andamento.
    if (expectedEpoch != generation) {
        uint32_t finishedUs = micros();
        xSemaphoreGive(lock);
        recordTiming(3, startedUs, acquiredUs, finishedUs);
        return 0;
    }
    uint8_t magnitude = static_cast<uint8_t>(pwm < 0 ? -pwm : pwm);
    // Mesmo um drive de duty zero pode habilitar DIR/enable.
    cutApplied = false;
    if (config.CytronDriver) {
        digitalWrite(DIR1_RL_ENABLE, pwm >= 0 ? HIGH : LOW);
        ledcWrite(PWM1_LPWM, magnitude);
    } else {
        digitalWrite(DIR1_RL_ENABLE, HIGH);
        if (pwm > 0) {
            ledcWrite(PWM2_RPWM, 0);
            ledcWrite(PWM1_LPWM, magnitude);
        } else {
            ledcWrite(PWM1_LPWM, 0);
            ledcWrite(PWM2_RPWM, magnitude);
        }
    }
    uint32_t finishedUs = micros();
    xSemaphoreGive(lock);
    recordTiming(3, startedUs, acquiredUs, finishedUs);
    return magnitude;
}
}
