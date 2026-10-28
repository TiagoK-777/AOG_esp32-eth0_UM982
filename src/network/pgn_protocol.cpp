#include "network/pgn_protocol.h"
#include <cmath>
namespace pgn {
void checksum(uint8_t* data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < length; ++i) sum += data[i];
    data[length - 1] = sum;
}
bool valid(const uint8_t* data, size_t length) {
    if (!data || length < 6 || data[0] != 0x80 || data[1] != 0x81 || data[2] != 0x7F) return false;
    size_t expected = static_cast<size_t>(data[4]) + 6;
    if (length != expected) return false;
    // AgIO envia este hello com checksum fixo 0x47 (não corresponde à soma).
    // A exceção é exclusiva do pacote de diagnóstico; comandos continuam estritos.
    if (length == 9 && data[3] == 200 && data[4] == 3 &&
        data[5] == 56 && data[6] == 0 && data[7] == 0 && data[8] == 0x47) return true;
    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < expected; ++i) sum += data[i];
    return sum == data[expected - 1];
}
bool decodeGuidance(const uint8_t* d, size_t len, GuidanceCommand& c) {
    if (!valid(d, len) || d[3] != 254 || d[4] != 8) return false;
    c.gpsSpeed = static_cast<float>(d[5] | (static_cast<uint16_t>(d[6]) << 8)) * 0.1f;
    c.guidanceStatus = d[7];
    c.steerAngleSetPoint = static_cast<float>(static_cast<int16_t>(d[8] | (static_cast<uint16_t>(d[9]) << 8))) * 0.01f;
    c.tram = d[10]; c.relay = d[11]; c.relayHi = d[12];
    c.valid = true;
    return true;
}
bool decodeSettings(const uint8_t* d, size_t len, Storage& s) {
    if (!valid(d, len) || d[3] != 252 || d[4] != 8 || d[9] == 0) return false;
    s.Kp = d[5]; s.highPWM = d[6]; s.minPWM = d[8];
    // Preserva a derivação histórica do lowPWM a partir do minPWM.
    s.lowPWM = static_cast<uint8_t>(static_cast<float>(s.minPWM) * 1.2f);
    s.steerSensorCounts = d[9];
    s.wasOffset = static_cast<int16_t>(d[10] | (static_cast<uint16_t>(d[11]) << 8));
    s.AckermanFix = static_cast<float>(d[12]) * 0.01f;
    return true;
}
bool decodeConfig(const uint8_t* d, size_t len, Setup& c) {
    if (!valid(d, len) || d[3] != 251 || d[4] != 8) return false;
    uint8_t first = d[5], second = d[8];
    c.InvertWAS = first & 1;
    c.IsRelayActiveHigh = (first >> 1) & 1;
    c.MotorDriveDirection = (first >> 2) & 1;
    c.SingleInputWAS = (first >> 3) & 1;
    c.CytronDriver = (first >> 4) & 1;
    c.SteerSwitch = (first >> 5) & 1;
    c.SteerButton = (first >> 6) & 1;
    c.ShaftEncoder = (first >> 7) & 1;
    c.PulseCountMax = d[6];
    c.IsDanfoss = second & 1;
    c.PressureSensor = 0;
    c.CurrentSensor = (second >> 2) & 1;
    c.IsUseY_Axis = (second >> 3) & 1;
    return true;
}
}
