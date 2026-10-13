#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// A mesma trava protege leitura e escrita, inclusive entre núcleos.
template<class T> class SharedValue {
public:
    T snapshot() const {
        portENTER_CRITICAL(&mux_);
        T copy = value_;
        portEXIT_CRITICAL(&mux_);
        return copy;
    }
    void publish(const T& value) {
        portENTER_CRITICAL(&mux_);
        value_ = value;
        portEXIT_CRITICAL(&mux_);
    }
private:
    T value_{};
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
