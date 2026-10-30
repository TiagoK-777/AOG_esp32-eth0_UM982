#pragma once
#include <cstdint>
namespace gps {
struct Stats {
    uint32_t rxBytes = 0, gga = 0, hpr = 0, panda = 0, queued = 0, buildFailed = 0;
    uint32_t nmeaErrors = 0, crcErrors = 0;
    uint32_t uartFifo = 0, uartBuffer = 0, uartFrame = 0, uartParity = 0;
    uint32_t backlogMax = 0, ggaGapUs = 0, pandaGapUs = 0;
};
Stats stats();
// Inicializa GPS e IMU uma vez, na task de navegação, antes de poll().
void setup();
void poll();
}
