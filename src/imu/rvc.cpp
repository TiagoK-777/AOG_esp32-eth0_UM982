/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
*/


#include <Arduino.h>
#include "imu/rvc.h"
#include "config/pinout.h"
#include "config/defines.h"
#include "diagnostics/log.h"
namespace imu {
namespace {
HardwareSerial SerialRVC(1);
uint32_t startedMs = 0;
bool reportedMissing = false;
void readBNO(ImuState& sample, const Setup& steerConfig)
{
    // BNO085 UART-RVC Protocol Parser
    // RVC packet format (19 bytes):
    // [0-1]: Header 0xAA 0xAA
    // [2]: Index (increments 0-255)
    // [3-4]: Yaw (int16, little-endian, 0.01° resolution)
    // [5-6]: Pitch (int16, little-endian, 0.01° resolution)
    // [7-8]: Roll (int16, little-endian, 0.01° resolution)
    // [9-10]: Accel X (int16, little-endian, 0.001g resolution)
    // [11-12]: Accel Y (int16, little-endian, 0.001g resolution)
    // [13-14]: Accel Z (int16, little-endian, 0.001g resolution)
    // [15-17]: Reserved
    // [18]: Checksum (Sum of bytes 2-17)

    static uint8_t rvcBuffer[RVC_PACKET_SIZE];
    static uint8_t bufferIndex = 0;

    // Removemos o estado VALIDATE pois a validação agora ocorre dentro de READ_DATA
    static enum {SYNC1, SYNC2, READ_DATA} state = SYNC1;
    static uint32_t checksumErrors = 0;

    // Angular velocity calculation (Ring Buffer 5-sample moving average)
    // 5 samples @ 100Hz = 50ms window: low lag, sufficient noise filtering
    /*
        Se no campo perceber o sample.gyroZ "nervoso" demais (improvável com o BNO085 RVC),
        basta subir para 8 amostras (yawDeltas) (fator 0.125, janela 80ms).
    */
    static int16_t prevYaw = 0;
    static int16_t yawDeltas[5] = {0};   // Circular buffer for last 5 deltas
    static uint8_t ringHead = 0;         // Current position in ring buffer
    static int32_t rollingSum = 0;       // Sum of all 5 deltas
    static bool firstYaw = true;
    static uint8_t lastIndex = 0;  // Track packet index to detect new packets
    static bool firstPacket = true;

    // --- NOVA LÓGICA DE BLOCK READ PARA O IMU ---
    int bytesAvailable = SerialRVC.available();

    if (bytesAvailable > 0)
    {
        // Limita a leitura a no máximo 4 pacotes (76 bytes) por ciclo para não prender o loop principal
        size_t bytesToRead = min((size_t)bytesAvailable, (size_t)76);
        uint8_t fastBuffer[76];

        // Puxa da UART de uma só vez para a Stack (SRAM) - Zero overhead de HAL
        size_t bytesRead = SerialRVC.read(fastBuffer, bytesToRead);

        // Processa tudo na velocidade da CPU
        for (size_t k = 0; k < bytesRead; k++)
        {
            uint8_t inByte = fastBuffer[k];

            switch(state)
            {
                case SYNC1:
                    if(inByte == RVC_HEADER) {
                        rvcBuffer[0] = inByte;
                        state = SYNC2;
                    }
                    break;

                case SYNC2:
                    if(inByte == RVC_HEADER) {
                        rvcBuffer[1] = inByte;
                        bufferIndex = 2;
                        state = READ_DATA;
                    } else {
                        // Not a valid header - restart
                        state = SYNC1;
                    }
                    break;

                case READ_DATA:
                    rvcBuffer[bufferIndex++] = inByte;

                    // Se o buffer encheu, validamos imediatamente
                    if(bufferIndex >= RVC_PACKET_SIZE) {

                        // Validate checksum (Sum of bytes 2-17)
                        uint8_t checksum = 0;
                        for(int i = 2; i < 18; i++) {
                            checksum += rvcBuffer[i];
                        }

                        if(checksum == rvcBuffer[18])
                        {
                            // Check if this is a new packet (Index byte increments 0-255)
                            uint8_t currentIndex = rvcBuffer[2];
                            // Only process if this is a NEW packet (Index changed)
                            if(firstPacket || currentIndex != lastIndex)
                            {
                                firstPacket = false;
                                lastIndex = currentIndex;

                                // Extract data (int16, little-endian)
                                int16_t rvcYaw = (int16_t)(rvcBuffer[3] | (rvcBuffer[4] << 8));
                                int16_t rvcPitch = (int16_t)(rvcBuffer[5] | (rvcBuffer[6] << 8));
                                int16_t rvcRoll = (int16_t)(rvcBuffer[7] | (rvcBuffer[8] << 8));

                                // Convert from 0.01° to degrees
                                double tempYaw = rvcYaw;    // In 0.01°
                                double tempPitch = rvcPitch;
                                double tempRoll = rvcRoll;

                                // Apply axis swap if configured (IsUseY_Axis swaps pitch/roll)
                                // Matches Teensy logic: swap happens when IsUseY_Axis=false (default)
                                if(!steerConfig.IsUseY_Axis)
                                {
                                    sample.roll = tempPitch;   // Pitch becomes Roll
                                    sample.pitch = tempRoll;   // Roll becomes Pitch
                                }
                                else
                                {
                                    sample.pitch = tempPitch;
                                    sample.roll = tempRoll;
                                }

                                // Apply roll inversion if configured
                                if(userSettings.invertRoll)
                                {
                                    sample.roll *= -1;
                                }

                                // Process Yaw (heading)
                                // Convert to radians for correctionHeading (used in fusion)
                                sample.correctionHeading = -(tempYaw / 100.0) * (PI / 180.0);  // 0.01° to radians, negated

                                // Convert yaw to 0.1° resolution (tenths of degree) for imuHandler
                                sample.yaw = (int16_t)(tempYaw / 10.0);  // 0.01° to 0.1°
                                if(sample.yaw < 0) sample.yaw += 3600;
                                if(sample.yaw >= 3600) sample.yaw -= 3600;

                                // Pitch and Roll: convert from 0.01° to 0.1° (divide by 10)
                                sample.pitch = sample.pitch / 10.0;  // 0.01° to 0.1°
                                sample.roll = sample.roll / 10.0;    // 0.01° to 0.1°

                                // Calculate angular velocity (yaw rate) from yaw deltas
                                // BNO085 RVC transmits at ~100Hz (hardware fixed rate)
                                // Loop frequency doesn't matter - we detect NEW packets by Index byte
                                // 20 new packets = 0.2s real time (20/100Hz)
                                if(firstYaw) {
                                    prevYaw = rvcYaw;
                                    firstYaw = false;
                                    sample.gyroZ = 0;
                                } else {
                                    // Calculate yaw delta (handle wraparound at ±180°)
                                    int32_t yawDelta = static_cast<int32_t>(rvcYaw) - prevYaw;
                                    if(yawDelta > 18000) yawDelta -= 36000;      // Wrapped from +180 to -180
                                    else if(yawDelta < -18000) yawDelta += 36000; // Wrapped from -180 to +180

                                    // Ring Buffer Moving Average (updates EVERY packet = 100Hz)
                                    // Remove oldest value from sum, add newest value
                                    rollingSum -= yawDeltas[ringHead];
                                    rollingSum += yawDelta;
                                    yawDeltas[ringHead] = yawDelta;

                                    // Advance ring buffer position (circular)
                                    ringHead++;
                                    if(ringHead >= 5) ringHead = 0; // Ajustar se necessário para 8 amostras (ringHead >= 8)

                                    // Calculate gyroZ from rolling sum (updates every packet!)
                                    // Formula: gyroZ = (avg_delta_per_packet) × (packets_per_sec) × (degrees_per_count)
                                    // gyroZ = (rollingSum/5) × 100Hz × 0.01°/count = rollingSum × 0.2 (°/s)
                                    sample.gyroZ = (double)rollingSum * 0.2; // Ajustar fator se necessário para 8 amostras (0.125 para 8 amostras)

                                    prevYaw = rvcYaw;
                                }
                            }  // End of "new packet" check

                            // Update watchdog (even for duplicate packets)
                            sample.lastRvcTime = millis();
                            sample.rvcPacketCount++;
                            sample.useBNO08x = true;
                        }
                        else
                        {
                            // Checksum failed
                            checksumErrors++;
                            if(checksumErrors % 100 == 0) {
                                AOG_LOG_PRINT("RVC checksum errors: ");
                                AOG_LOG_PRINTLN(checksumErrors);
                            }
                        }

                        // Reset state machine immediately for next packet
                        state = SYNC1;
                        bufferIndex = 0;
                    }
                    break;
            } // Fim do switch
        } // Fim do loop for
    } // Fim do if (bytesAvailable > 0)
}
}
void setup() {
    SerialRVC.setRxBufferSize(256);
    SerialRVC.begin(115200, SERIAL_8N1, RVC_RX_PIN, -1);
    startedMs = millis();
    AOG_LOG_PRINTLN("BNO085 UART-RVC initialized on GPIO2");
}
void poll(ImuState& sample, const Setup& config) {
    uint32_t before = sample.rvcPacketCount;
    readBNO(sample, config);
    if (before == 0 && sample.rvcPacketCount != 0) AOG_LOG_PRINTLN("RVC sync found!");
    if (!reportedMissing && sample.rvcPacketCount == 0 && millis() - startedMs >= 2000) {
        AOG_LOG_PRINTLN("WARNING: No RVC packets detected! Check BNO085 connection.");
        reportedMissing = true;
    }
    // A UART continua sendo atendida para permitir recuperação da IMU.
}
}
