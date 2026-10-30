#pragma once
#include <Arduino.h>

#ifndef AOG_DEBUG_LOGS
#define AOG_DEBUG_LOGS 0
#endif

#if AOG_DEBUG_LOGS
#define AOG_LOG_PRINT(...) do { Serial.print(__VA_ARGS__); } while (0)
#define AOG_LOG_PRINTLN(...) do { Serial.println(__VA_ARGS__); } while (0)
#define AOG_LOG_PRINTF(...) do { Serial.printf(__VA_ARGS__); } while (0)
#else
#define AOG_LOG_PRINT(...) do {} while (0)
#define AOG_LOG_PRINTLN(...) do {} while (0)
#define AOG_LOG_PRINTF(...) do {} while (0)
#endif
