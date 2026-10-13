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
void cut() {
    digitalWrite(DIR1_RL_ENABLE, LOW);
    ledcWrite(PWM1_LPWM, 0);
    ledcWrite(PWM2_RPWM, 0);
}
}
namespace motor {
bool setup() {
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
    xSemaphoreTake(lock, portMAX_DELAY);
    uint32_t value = generation;
    xSemaphoreGive(lock);
    return value;
}
void stop() {
    if (!lock) return;
    xSemaphoreTake(lock, portMAX_DELAY);
    ++generation;
    cut();
    xSemaphoreGive(lock);
}
uint8_t drive(int16_t pwm, const Setup& config, uint32_t expectedEpoch) {
    if (!lock) return 0;
    xSemaphoreTake(lock, portMAX_DELAY);
    // Um corte concorrente invalida o cálculo que já estava em andamento.
    if (expectedEpoch != generation) {
        xSemaphoreGive(lock);
        return 0;
    }
    uint8_t magnitude = static_cast<uint8_t>(pwm < 0 ? -pwm : pwm);
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
    xSemaphoreGive(lock);
    return magnitude;
}
}
