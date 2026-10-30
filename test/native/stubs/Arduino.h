#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <deque>
#include <vector>
using std::abs;
using std::min;
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr int HIGH = 1, LOW = 0, OUTPUT = 1, INPUT = 0;
constexpr int SERIAL_8N1 = 0, ADC_11db = 0;
inline uint32_t fakeMillis = 0;
inline uint32_t fakeMicrosOffset = 0, fakeLedcWriteUs = 0;
inline int fakePins[40]{};
inline uint32_t fakePwm[40]{};
inline uint32_t fakeLedcWrites[40]{};
inline int fakeLedcFailPin = -1;
enum hardwareSerial_error_t { UART_NO_ERROR, UART_BREAK_ERROR, UART_BUFFER_FULL_ERROR,
    UART_FIFO_OVF_ERROR, UART_FRAME_ERROR, UART_PARITY_ERROR };
inline void (*fakeUartErrorCallback[3])(hardwareSerial_error_t){};
inline uint32_t millis() { return fakeMillis; }
inline uint32_t micros() { return fakeMillis * 1000 + fakeMicrosOffset; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) { fakePins[pin] = value; }
inline int digitalRead(int pin) { return fakePins[pin]; }
inline bool ledcAttach(int, uint32_t, uint8_t) { return true; }
inline bool ledcWrite(int pin, uint32_t value) {
    ++fakeLedcWrites[pin];
    fakeMicrosOffset += fakeLedcWriteUs;
    if (pin == fakeLedcFailPin) return false;
    fakePwm[pin] = value;
    return true;
}
inline void analogReadResolution(int) {}
inline void analogSetAttenuation(int) {}
inline int analogRead(int) { return 1992; }
inline char* dtostrf(double value, signed char width, unsigned char precision, char* output) {
    std::sprintf(output, "%*.*f", static_cast<int>(width), static_cast<int>(precision), value);
    return output;
}
inline char* itoa(int value, char* output, int) { std::sprintf(output, "%d", value); return output; }
inline std::deque<uint8_t> fakeUartRx[3];
inline std::vector<uint8_t> fakeUartTx[3];
class HardwareSerial {
public:
    explicit HardwareSerial(int id) : id_(id) {}
    void begin(int, int = 0, int = -1, int = -1) {}
    size_t setRxBufferSize(size_t size) { return size; }
    void onReceiveError(void (*callback)(hardwareSerial_error_t)) { fakeUartErrorCallback[id_] = callback; }
    size_t setTxBufferSize(size_t size) { return size; }
    int available() { return fakeUartRx[id_].size(); }
    int availableForWrite() { return 256; }
    size_t read(uint8_t* bytes, size_t count) {
        count = std::min(count, fakeUartRx[id_].size());
        for (size_t i = 0; i < count; ++i) { bytes[i] = fakeUartRx[id_].front(); fakeUartRx[id_].pop_front(); }
        return count;
    }
    size_t write(const uint8_t* bytes, size_t count) {
        fakeUartTx[id_].insert(fakeUartTx[id_].end(), bytes, bytes + count);
        return count;
    }
    template<class T> void print(const T&) {}
    template<class T> void println(const T&) {}
private:
    int id_;
};
inline HardwareSerial Serial(0);
inline HardwareSerial Serial2(2);
