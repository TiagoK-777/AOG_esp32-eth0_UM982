#pragma once
#include <mutex>
#include <cstdint>
using portMUX_TYPE = std::mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(mux) (mux)->lock()
#define portEXIT_CRITICAL(mux) (mux)->unlock()
constexpr uint32_t portMAX_DELAY = UINT32_MAX;
