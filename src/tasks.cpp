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
#include "config/settings.h"
#include "state/steer_state.h"
#include "state/guidance_state.h"
#include "diagnostics/timing.h"
namespace {
enum TaskId { Control, Navigation, Network, Safety, Count };
diagnostics::TimingCounters telemetry[Count];
uint32_t reportMs = 0;
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
    for (;;) {
        uint32_t started = micros();
        // wake é o tick previsto pelo vTaskDelayUntil, inclusive após overruns.
        uint32_t lateMs = (xTaskGetTickCount() - wake) * portTICK_PERIOD_MS;
        task.poll();
        uint32_t duration = micros() - started;
        telemetry[task.id].record(started, duration, lateMs, task.periodMs * 1000);
        esp_task_wdt_reset();
        // Espera cooperativa do RTOS; nenhuma task ocupa CPU enquanto aguarda.
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(task.periodMs));
    }
}
}
namespace tasks {
bool create() {
    reportMs = millis();
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
        {"navigation", navigationStack, sizeof(navigationStack), 3, 0},
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
    uint32_t now = millis();
    uint32_t elapsed = now - reportMs;
    reportMs = now;
    ::diagnostics::Timing samples[Count];
    for (uint8_t i = 0; i < Count; ++i) samples[i] = telemetry[i].take();
    auto cycle = autosteer::takeCycleTiming();
    auto adc = autosteer::takeAdcTiming();
    auto motorTiming = motor::takeTiming();
    auto gpsValue = gps::stats();
    network::Stats net = network::stats();
    static uint32_t previousGga = 0, previousPanda = 0, previousSent = 0;
    auto hz = [elapsed](uint32_t delta) { return elapsed ? delta * 1000.0 / elapsed : 0.0; };
    double ggaHz = hz(gpsValue.gga - previousGga);
    double pandaHz = hz(gpsValue.panda - previousPanda);
    double sentHz = hz(net.gpsSent - previousSent);
    previousGga = gpsValue.gga; previousPanda = gpsValue.panda; previousSent = net.gpsSent;
    Serial.printf("DIAG uptime=%lu ms window=%lu ms; peaks since previous report\n",
        static_cast<unsigned long>(now), static_cast<unsigned long>(elapsed));
    const char* names[] = {"control", "navigation", "network", "safety"};
    for (uint8_t i = 0; i < Count; ++i) {
        const auto& value = samples[i];
        unsigned stack = handles[i] ? uxTaskGetStackHighWaterMark(handles[i]) : 0;
        uint32_t hz = elapsed ? static_cast<uint64_t>(value.calls) * 1000 / elapsed : 0;
        Serial.printf("TASK %s: %lu Hz, peak %lu us, gapMax=%lu us lateMax=%lu ms late=%lu over=%lu stack=%u bytes\n",
            names[i], static_cast<unsigned long>(hz), static_cast<unsigned long>(value.peakUs),
            static_cast<unsigned long>(value.gapUs), static_cast<unsigned long>(value.lateMs),
            static_cast<unsigned long>(value.lateStarts), static_cast<unsigned long>(value.overBudget), stack);
    }
    Serial.printf("CYCLE n=%lu peak=%lu us gapMax=%lu us lateMax=%lu ms late=%lu over25ms=%lu | ADS n=%lu peak=%lu us\n",
        static_cast<unsigned long>(cycle.calls), static_cast<unsigned long>(cycle.peakUs),
        static_cast<unsigned long>(cycle.gapUs), static_cast<unsigned long>(cycle.lateMs),
        static_cast<unsigned long>(cycle.lateStarts), static_cast<unsigned long>(cycle.overBudget),
        static_cast<unsigned long>(adc.calls), static_cast<unsigned long>(adc.peakUs));
    const char* motorNames[] = {"stop/control", "stop/safety", "stop/other", "drive", "epoch"};
    for (uint8_t i = 0; i < motor::TIMING_COUNT; ++i) {
        const auto& value = motorTiming.operations[i];
        Serial.printf("MOTOR %s: n=%lu totalMax=%lu us waitMax=%lu us bodyMax=%lu us\n",
            motorNames[i], static_cast<unsigned long>(value.calls), static_cast<unsigned long>(value.totalUs),
            static_cast<unsigned long>(value.waitUs), static_cast<unsigned long>(value.bodyUs));
    }
    SteerStatus status = state::steerSnapshot();
    GuidanceCommand command = state::guidanceSnapshot();
    Serial.printf("GPS gga=%lu (%.2f Hz) hpr=%lu panda=%lu (%.2f Hz) queued=%lu tx=%lu (%.2f Hz) gapGga=%lu us gapPanda=%lu us gapTx=%lu us\n",
        static_cast<unsigned long>(gpsValue.gga), ggaHz, static_cast<unsigned long>(gpsValue.hpr),
        static_cast<unsigned long>(gpsValue.panda), pandaHz, static_cast<unsigned long>(gpsValue.queued),
        static_cast<unsigned long>(net.gpsSent), sentHz, static_cast<unsigned long>(gpsValue.ggaGapUs),
        static_cast<unsigned long>(gpsValue.pandaGapUs), static_cast<unsigned long>(net.gpsGapUs));
    Serial.printf("GPS RX bytes=%lu backlogMax=%lu nmeaErr=%lu crcErr=%lu fifoOvf=%lu rxFull=%lu frameErr=%lu parityErr=%lu buildFail=%lu txFail=%lu\n",
        static_cast<unsigned long>(gpsValue.rxBytes), static_cast<unsigned long>(gpsValue.backlogMax),
        static_cast<unsigned long>(gpsValue.nmeaErrors), static_cast<unsigned long>(gpsValue.crcErrors),
        static_cast<unsigned long>(gpsValue.uartFifo), static_cast<unsigned long>(gpsValue.uartBuffer),
        static_cast<unsigned long>(gpsValue.uartFrame), static_cast<unsigned long>(gpsValue.uartParity),
        static_cast<unsigned long>(gpsValue.buildFailed), static_cast<unsigned long>(net.gpsFailed));
    Serial.printf("CONTROL cycles=%lu peak=%lu us | TX drops=%lu RTK drops=%lu invalid=%lu sendFail=%lu heap=%u\n",
        static_cast<unsigned long>(status.cycles), static_cast<unsigned long>(status.maxCycleUs),
        static_cast<unsigned long>(udp_tx::droppedPackets()), static_cast<unsigned long>(rtcm::droppedPackets()),
        static_cast<unsigned long>(net.invalidPackets), static_cast<unsigned long>(net.sendFailures),
        static_cast<unsigned>(ESP.getFreeHeap()));
    ConfigSnapshot values = config::snapshot();
    const char* mode = values.steerConfig.SteerSwitch ? "switch" : values.steerConfig.SteerButton ? "button" : "none";
    Serial.printf("STEER available=%u engaged=%u mode=%s switch=%u guidance=%u speed=%.1f pwm=%u\n",
        static_cast<unsigned>(status.available), static_cast<unsigned>(status.engaged), mode,
        static_cast<unsigned>(status.steerSwitch), static_cast<unsigned>(command.guidanceStatus),
        static_cast<double>(command.gpsSpeed), static_cast<unsigned>(status.pwmDisplay));
}
}
