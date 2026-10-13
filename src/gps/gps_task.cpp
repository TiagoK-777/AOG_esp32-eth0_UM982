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
}
void setup() {
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);
    parser.addHandler("G-HPR", HPR_Handler);
    SerialGPS->setRxBufferSize(1024);
    SerialGPS->setTxBufferSize(2048);
    SerialGPS->begin(460800, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
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

        if (bytesAvailable > 0)
        {
            // Limite para não prender o loop principal caso o buffer físico esteja cheio (ex: 128 ou 256 bytes)
            constexpr size_t MAX_RAW_BYTES_PER_CYCLE = 128;
            size_t bytesToRead = min((size_t)bytesAvailable, MAX_RAW_BYTES_PER_CYCLE);

            uint8_t rxBuffer[128]; // Buffer temporário na Stack
            size_t bytesRead = SerialGPS->read(rxBuffer, bytesToRead);

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

        if (bytesAvailable > 0)
        {
            // Puxa apenas o que já está garantido na memória da UART
            size_t bytesToRead = min((size_t)bytesAvailable, MAX_GPS_BYTES_PER_CYCLE);

            uint8_t rxBuffer[64];
            size_t bytesRead = SerialGPS->read(rxBuffer, bytesToRead);

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
}
}
