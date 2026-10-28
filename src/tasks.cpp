#include <Arduino.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "tasks.h"
#include "autosteer/autosteer_task.h"
#include "autosteer/safety.h"
#include "gps/gps.h"
#include "network/udp_rx.h"
#include "network/udp_tx.h"
#include "network/rtcm_buffer.h"
#include "hardware/motor_pwm.h"
#include "config/defines.h"
#include "state/steer_state.h"
#include "state/shared_value.h"
namespace {
enum TaskId { Control, Navigation, Network, Safety, Count };
struct Telemetry { uint32_t hz = 0, maxUs = 0; };
SharedValue<Telemetry> telemetry[Count];
TaskHandle_t handles[Count]{};
StaticTask_t taskStorage[Count];
StackType_t controlStack[4096], navigationStack[4096], networkStack[4096], safetyStack[2048];
struct Worker { TaskId id; void (*poll)(); uint32_t periodMs; };
Worker workers[] = {
    {Control, autosteer::poll, 1}, {Navigation, gps::poll, 1},
    {Network, network::poll, 1}, {Safety, safety::poll, SAFETY_PERIOD_MS}
};
void worker(void* argument) {
    const Worker& task = *static_cast<Worker*>(argument);
    if (esp_task_wdt_add(nullptr) != ESP_OK) {
        Serial.println("Failed to register task watchdog");
        motor::stop();
        vTaskSuspend(nullptr);
    }
    TickType_t wake = xTaskGetTickCount();
    uint32_t window = millis(), iterations = 0, peakUs = 0;
    for (;;) {
        uint32_t started = micros();
        task.poll();
        uint32_t duration = micros() - started;
        if (duration > peakUs) peakUs = duration;
        ++iterations;
        uint32_t elapsed = millis() - window;
        if (elapsed >= 1000) {
            telemetry[task.id].publish({iterations * 1000 / elapsed, peakUs});
            window = millis(); iterations = 0; peakUs = 0;
        }
        esp_task_wdt_reset();
        // Espera cooperativa do RTOS; nenhuma task ocupa CPU enquanto aguarda.
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(task.periodMs));
    }
}
}
namespace tasks {
bool create() {
    esp_task_wdt_config_t watchdog{};
    watchdog.timeout_ms = 2000;
    watchdog.idle_core_mask = 0;
    watchdog.trigger_panic = true;
    esp_err_t error = esp_task_wdt_init(&watchdog);
    if (error == ESP_ERR_INVALID_STATE) error = esp_task_wdt_reconfigure(&watchdog);
    if (error != ESP_OK) return false;
    struct Definition { const char* name; StackType_t* stack; uint32_t bytes; UBaseType_t priority; BaseType_t core; };
    const Definition definitions[] = {
        {"autosteer", controlStack, sizeof(controlStack), 5, 1},
        // Afinidade escolhida após spikes com navegação/rede no núcleo 0 no ESP32-S3.
        {"navigation", navigationStack, sizeof(navigationStack), 3, 1},
        {"network", networkStack, sizeof(networkStack), 2, 0},
        {"safety", safetyStack, sizeof(safetyStack), 6, 1}
    };
    for (uint8_t i = 0; i < Count; ++i) {
        const auto& definition = definitions[i];
        handles[i] = xTaskCreateStaticPinnedToCore(worker, definition.name, definition.bytes,
            &workers[i], definition.priority, definition.stack, &taskStorage[i], definition.core);
        if (!handles[i]) {
            for (uint8_t j = 0; j < i; ++j) {
                esp_task_wdt_delete(handles[j]);
                vTaskDelete(handles[j]);
                handles[j] = nullptr;
            }
            motor::stop();
            return false;
        }
    }
    return true;
}
void diagnostics() {
    const char* names[] = {"control", "navigation", "network", "safety"};
    for (uint8_t i = 0; i < Count; ++i) {
        Telemetry value = telemetry[i].snapshot();
        unsigned stack = handles[i] ? uxTaskGetStackHighWaterMark(handles[i]) : 0;
        Serial.printf("TASK %s: %lu Hz, peak %lu us, stack %u bytes\n", names[i],
            static_cast<unsigned long>(value.hz), static_cast<unsigned long>(value.maxUs), stack);
    }
    SteerStatus status = state::steerSnapshot();
    network::Stats net = network::stats();
    Serial.printf("CONTROL cycles=%lu peak=%lu us | TX drops=%lu RTK drops=%lu invalid=%lu sendFail=%lu heap=%u\n",
        static_cast<unsigned long>(status.cycles), static_cast<unsigned long>(status.maxCycleUs),
        static_cast<unsigned long>(udp_tx::droppedPackets()), static_cast<unsigned long>(rtcm::droppedPackets()),
        static_cast<unsigned long>(net.invalidPackets), static_cast<unsigned long>(net.sendFailures), ESP.getFreeHeap());
}
}
