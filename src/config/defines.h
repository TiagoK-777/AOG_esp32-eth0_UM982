#pragma once
#include <cstddef>
#include <cstdint>

constexpr uint16_t PORT_GPS = 5120;
constexpr uint16_t PORT_NTRIP = 2233;
constexpr uint16_t PORT_AUTOSTEER = 8888;
constexpr uint16_t PORT_DESTINATION = 9999;
constexpr uint32_t CONTROL_PERIOD_MS = 25;
constexpr uint32_t COMMAND_TIMEOUT_MS = 2500;
constexpr uint32_t CONTROL_STALE_MS = 100;
constexpr uint32_t SAFETY_PERIOD_MS = 20;
constexpr uint32_t MAINTENANCE_PERIOD_MS = 10;
constexpr uint16_t I2C_TIMEOUT_MS = 3;
constexpr float LOW_HIGH_DEGREES = 3.0f;
constexpr float WAS_ANGLE_MAX = 45.0f;
constexpr uint8_t PWM_Frequency = 0;
constexpr int EEP_Ident = 2400;
constexpr uint8_t RVC_HEADER = 0xAA;
constexpr size_t RVC_PACKET_SIZE = 19;
constexpr double twoPI = 6.28318530717958647692;
constexpr double PIBy2 = 1.57079632679489661923;
constexpr size_t UDP_TX_QUEUE_DEPTH = 8;
constexpr size_t UDP_TX_MAX_PAYLOAD = 256;
constexpr size_t RTCM_BUFFER_SIZE = 2048;
