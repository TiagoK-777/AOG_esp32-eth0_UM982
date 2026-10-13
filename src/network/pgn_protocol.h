#pragma once
#include <cstddef>
#include <cstdint>
#include "config/settings.h"
#include "state/guidance_state.h"
namespace pgn {
bool valid(const uint8_t* data, size_t length);
bool decodeGuidance(const uint8_t* data, size_t length, GuidanceCommand& command);
bool decodeSettings(const uint8_t* data, size_t length, Storage& settings);
bool decodeConfig(const uint8_t* data, size_t length, Setup& config);
void checksum(uint8_t* data, size_t length);
}
