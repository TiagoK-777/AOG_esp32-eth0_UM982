#pragma once
#include "FreeRTOS.h"
using StaticSemaphore_t = std::mutex;
using SemaphoreHandle_t = std::mutex*;
inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* storage) { return storage; }
inline bool xSemaphoreTake(SemaphoreHandle_t lock, uint32_t) { lock->lock(); return true; }
inline bool xSemaphoreGive(SemaphoreHandle_t lock) { lock->unlock(); return true; }
