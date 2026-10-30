/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
*/


#include <Arduino.h>
#include "gps/gps.h"
#include "gps/context.h"
#include "imu/rvc.h"
#include "config/settings.h"
#include "config/pinout.h"
#include "network/udp_tx.h"
#include "network/rtcm_buffer.h"
#include "state/shared_value.h"
#include <atomic>
namespace gps {
Context context;
ImuState imu;
NMEAParser<4> parser;
SimpleKalmanFilter rollFilter(userSettings.rollMEA, userSettings.rollEST, userSettings.rollQ);
SimpleKalmanFilter headingFilter(userSettings.headingMEA, userSettings.headingEST, userSettings.headingQ);
namespace {
HardwareSerial* SerialGPS = &Serial2;
bool gotCR = false, gotLF = false, gotDollar = false;
char msgBuf[254];
int msgBufLen = 0;
Stats counters;
SharedValue<Stats> published;
std::atomic<uint32_t> fifoErrors{0}, bufferErrors{0}, frameErrors{0}, parityErrors{0};
uint32_t lastGgaUs = 0, lastPandaUs = 0;
void uartError(hardwareSerial_error_t error) {
    if (error == UART_FIFO_OVF_ERROR) ++fifoErrors;
    else if (error == UART_BUFFER_FULL_ERROR) ++bufferErrors;
    else if (error == UART_FRAME_ERROR) ++frameErrors;
    else if (error == UART_PARITY_ERROR) ++parityErrors;
}
void observeBacklog(int bytes) {
    if (bytes > 0 && static_cast<uint32_t>(bytes) > counters.backlogMax) counters.backlogMax = bytes;
}
}
Stats stats() {
    Stats value = published.snapshot();
    value.uartFifo = fifoErrors.load(); value.uartBuffer = bufferErrors.load();
    value.uartFrame = frameErrors.load(); value.uartParity = parityErrors.load();
    return value;
}
void recordGga() {
    uint32_t now = micros();
    if (counters.gga && now - lastGgaUs > counters.ggaGapUs) counters.ggaGapUs = now - lastGgaUs;
    lastGgaUs = now;
    ++counters.gga;
}
void recordHpr() { ++counters.hpr; }
void recordNmeaError(bool crc) { ++counters.nmeaErrors; if (crc) ++counters.crcErrors; }
void recordPanda(bool built, bool queued) {
    if (!built) { ++counters.buildFailed; return; }
    uint32_t now = micros();
    if (counters.panda && now - lastPandaUs > counters.pandaGapUs) counters.pandaGapUs = now - lastPandaUs;
    lastPandaUs = now;
    ++counters.panda;
    if (queued) ++counters.queued;
}
void setup() {
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);
    parser.addHandler("G-HPR", HPR_Handler);
    SerialGPS->setRxBufferSize(1024);
    SerialGPS->setTxBufferSize(2048);
    SerialGPS->begin(460800, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    SerialGPS->onReceiveError(uartError);
    ::imu::setup();
}
void poll() {
    ::imu::poll(imu, config::snapshot().steerConfig);
    state::publishImu(imu);
    if (userSettings.udpPassthrough)
    {
        // Modo raw: captura sentença completa NMEA e envia via UDP/USB
        // Otimizado: Leitura em bloco para eliminar o overhead de chamadas sucessivas à UART
        int bytesAvailable = SerialGPS->available();
        observeBacklog(bytesAvailable);

        if (bytesAvailable > 0)
        {
            // Limite para não prender o loop principal caso o buffer físico esteja cheio (ex: 128 ou 256 bytes)
            constexpr size_t MAX_RAW_BYTES_PER_CYCLE = 128;
            size_t bytesToRead = min((size_t)bytesAvailable, MAX_RAW_BYTES_PER_CYCLE);

            uint8_t rxBuffer[MAX_RAW_BYTES_PER_CYCLE]; // Buffer temporário na Stack
            size_t bytesRead = SerialGPS->read(rxBuffer, bytesToRead);
            counters.rxBytes += bytesRead;

            // Processa na velocidade máxima do núcleo (RAM interna)
            for (size_t i = 0; i < bytesRead; i++)
            {
                char incoming = (char)rxBuffer[i];

                switch (incoming) {
                    case '$':
                    case '!':  // NMEA também aceita ! como início
                        // Nova sentença: reseta buffer
                        msgBufLen = 0;
                        gotCR = false;
                        gotLF = false;
                        gotDollar = true;
                        msgBuf[msgBufLen++] = incoming;
                        break;

                    case '\r':
                        if (gotDollar && msgBufLen < sizeof(msgBuf)) {
                            msgBuf[msgBufLen++] = incoming;
                            gotCR = true;
                        }
                        break;

                    case '\n':
                        if (gotDollar && gotCR && msgBufLen < sizeof(msgBuf)) {
                            msgBuf[msgBufLen++] = incoming;
                            gotLF = true;
                        }
                        break;

                    default:
                        if (gotDollar && msgBufLen < sizeof(msgBuf)) {
                            msgBuf[msgBufLen++] = incoming;
                        }
                        break;
                }

                // Sentença completa: envia
                if (gotCR && gotLF) {
                    if (userSettings.sendUSB) {
                        Serial.write((const uint8_t*)msgBuf, msgBufLen);
                    }
                    udp_tx::enqueue(udp_tx::Channel::GPS, reinterpret_cast<const uint8_t*>(msgBuf), msgBufLen);

                    context.blink = !context.blink;

                    // Reseta para próxima sentença
                    gotCR = false;
                    gotLF = false;
                    gotDollar = false;
                    msgBufLen = 0;
                }

                // Proteção: se estourar buffer sem fechar, descarta
                if (msgBufLen >= sizeof(msgBuf)) {
                    gotCR = false;
                    gotLF = false;
                    gotDollar = false;
                    msgBufLen = 0;
                }
            }
        }
    }
    else
    {
        // Modo parser: alimenta o NMEAParser a partir da leitura rápida da Stack
        // Limite por ciclo distribui o processamento sem corromper o parser (state-machine stateful)
        constexpr size_t MAX_GPS_BYTES_PER_CYCLE = 64;
        int bytesAvailable = SerialGPS->available();
        observeBacklog(bytesAvailable);

        if (bytesAvailable > 0)
        {
            // Puxa apenas o que já está garantido na memória da UART
            size_t bytesToRead = min((size_t)bytesAvailable, MAX_GPS_BYTES_PER_CYCLE);

            uint8_t rxBuffer[MAX_GPS_BYTES_PER_CYCLE];
            size_t bytesRead = SerialGPS->read(rxBuffer, bytesToRead);
            counters.rxBytes += bytesRead;

            // Injeta no Glinnes/NMEAParser rodando integralmente na SRAM
            for (size_t i = 0; i < bytesRead; i++)
            {
                parser << (char)rxBuffer[i];
            }
        }
    }


    bool ready = context.dualReadyGGA && (!userSettings.baseLineCheck || context.dualReadyRelPos);
    if (ready) {
        imuHandler();
        BuildNmea();
        context.dualReadyGGA = false;
        context.dualReadyRelPos = false;
    }
    int room = SerialGPS->availableForWrite();
    if (room > 0) {
        uint8_t bytes[128];
        size_t capacity = static_cast<size_t>(room) < sizeof(bytes) ? room : sizeof(bytes);
        size_t count = rtcm::read(bytes, capacity);
        if (count) SerialGPS->write(bytes, count);
    }
    published.publish(counters);
}
}
