#pragma once
#include <cstdint>
#include <freertos/FreeRTOS.h>

namespace diagnostics {
struct Timing {
    uint32_t calls = 0;
    uint32_t peakUs = 0;
    uint32_t gapUs = 0;
    uint32_t lateMs = 0;
    uint32_t lateStarts = 0;
    uint32_t overBudget = 0;
};

// O consumidor retira uma janela completa; nenhum pico intermediário é perdido.
class TimingCounters {
public:
    void record(uint32_t startUs, uint32_t durationUs, uint32_t lateMs, uint32_t budgetUs) {
        portENTER_CRITICAL(&mux_);
        ++value_.calls;
        if (durationUs > value_.peakUs) value_.peakUs = durationUs;
        if (hasPrevious_) {
            uint32_t gap = startUs - previousUs_;
            if (gap > value_.gapUs) value_.gapUs = gap;
        }
        previousUs_ = startUs;
        hasPrevious_ = true;
        if (lateMs > value_.lateMs) value_.lateMs = lateMs;
        if (lateMs) ++value_.lateStarts;
        if (durationUs > budgetUs) ++value_.overBudget;
        portEXIT_CRITICAL(&mux_);
    }
    Timing take() {
        portENTER_CRITICAL(&mux_);
        Timing copy = value_;
        value_ = {};
        portEXIT_CRITICAL(&mux_);
        return copy;
    }
private:
    Timing value_{};
    uint32_t previousUs_ = 0;
    bool hasPrevious_ = false;
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
}
