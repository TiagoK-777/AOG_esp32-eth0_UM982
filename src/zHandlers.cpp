/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include "GlobalVariables.h"

// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG
char vtgHeading[12] = { };
char speedKnots[10] = { };

// HPR
char umHeading[8];
char umRoll[8];
int solQuality;

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

// If odd characters showed up.
void errorHandler()
{
  // Error handling can be added here if needed
}

void GGA_Handler() //Rec'd GGA
{
    // fix time
    parser.getArg(0, fixTime);

    // latitude
    parser.getArg(1, latitude);
    parser.getArg(2, latNS);

    // longitude
    parser.getArg(3, longitude);
    parser.getArg(4, lonEW);

    // fix quality
    parser.getArg(5, fixQuality);

    // satellite #
    parser.getArg(6, numSats);

    // HDOP
    parser.getArg(7, HDOP);

    // altitude
    parser.getArg(8, altitude);

    // time of last DGPS update
    parser.getArg(12, ageDGPS);

    if (blink)
    {
        //digitalWrite(GGAReceivedLED, HIGH);  //Commented to save ESP32 pins
    }
    else
    {
        //digitalWrite(GGAReceivedLED, LOW);   //Commented to save ESP32 pins
    }

    blink = !blink;
    GGA_Available = true;

    dualReadyGGA = true;
   
    gpsReadyTime = millis();    //Used for GGA timeout (LED's ETC) 
}

void VTG_Handler()
{
  // vtg heading
  parser.getArg(0, vtgHeading);

  // vtg Speed knots
  parser.getArg(4, speedKnots);
}

//UM982 Support
void HPR_Handler()
{ 
  dualReadyRelPos = true;
  //digitalWrite(GPSRED_LED, LOW);   //Turn red GPS LED OFF (we are now in dual mode so green LED) - Commented to save ESP32 pins

  // HPR Heading
  parser.getArg(1, umHeading);
  heading = atof(umHeading);
  if ( filterHeading )
    {
      float tempHeading;
      tempHeading = headingFilter.updateEstimate(heading);
      heading = tempHeading;
    }

  // HPR Substitute pitch for roll
  if ( parser.getArg(2, umRoll) )
  {
    rollDual = atof(umRoll);
    //digitalWrite(GPSGREEN_LED, HIGH);   //Turn green GPS LED ON - Commented to save ESP32 pins
    if ( filterRoll )
      {
        float tempRoll;
        tempRoll = rollFilter.updateEstimate(rollDual);
        rollDual = tempRoll;
      }
  }
  else
  {
    //digitalWrite(GPSGREEN_LED, blink);  //Flash the green GPS LED - Commented to save ESP32 pins
  }

  // Solution quality factor
  parser.getArg(4, solQuality);

  if (solQuality >= 4)
    {
       if (useBNO08x)
       {
           if (baseLineCheck)
           {
               imuDualDelta();         //Find the error between latest IMU reading and this dual message
              dualReadyRelPos = false;  //RelPos ready is false because we just saved the error for running from the IMU
           }

       }
       else
       {
           imuHandler();             //No IMU so use dual data direct
           dualReadyRelPos = true;   //RelPos ready is true so PAOGI will send when the GGA is also ready
       }
    }
}

void readBNO()
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
    
    // Angular velocity calculation (Ring Buffer 20-sample moving average)
    // Updates every packet (100Hz) instead of every 20 packets (5Hz)
    static int16_t prevYaw = 0;
    static int16_t yawDeltas[20] = {0};  // Circular buffer for last 20 deltas
    static uint8_t ringHead = 0;         // Current position in ring buffer
    static int32_t rollingSum = 0;       // Sum of all 20 deltas
    static bool firstYaw = true;
    static uint8_t lastIndex = 0;  // Track packet index to detect new packets
    static bool firstPacket = true;

    // Process all available bytes
    while(SerialRVC.available() > 0)
    {
        uint8_t inByte = SerialRVC.read();

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
                        if(firstPacket) {
                            lastIndex = currentIndex;
                            firstPacket = false;
                        }
                        
                        // Only process if this is a NEW packet (Index changed)
                        if(currentIndex != lastIndex)
                        {
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
                            if(steerConfig.IsUseY_Axis)
                            {
                                roll = tempPitch;   // Pitch becomes Roll
                                pitch = tempRoll;   // Roll becomes Pitch
                            }
                            else
                            {
                                pitch = tempPitch;
                                roll = tempRoll;
                            }

                            // Apply roll inversion if configured
                            if(invertRoll)
                            {
                                roll *= -1;
                            }

                            // Process Yaw (heading)
                            // Convert to radians for correctionHeading (used in fusion)
                            correctionHeading = -(tempYaw / 100.0) * (PI / 180.0);  // 0.01° to radians, negated

                            // Convert yaw to 0.1° resolution (tenths of degree) for imuHandler
                            yaw = (int16_t)(tempYaw / 10.0);  // 0.01° to 0.1°
                            if(yaw < 0) yaw += 3600;
                            if(yaw >= 3600) yaw -= 3600;

                            // Pitch and Roll: convert from 0.01° to 0.1° (divide by 10)
                            pitch = pitch / 10.0;  // 0.01° to 0.1°
                            roll = roll / 10.0;    // 0.01° to 0.1°

                            // Calculate angular velocity (yaw rate) from yaw deltas
                            // BNO085 RVC transmits at ~100Hz (hardware fixed rate)
                            // Loop frequency doesn't matter - we detect NEW packets by Index byte
                            // 20 new packets = 0.2s real time (20/100Hz)
                            if(firstYaw) {
                                prevYaw = rvcYaw;
                                firstYaw = false;
                                gyroZ = 0;
                            } else {
                                // Calculate yaw delta (handle wraparound at ±180°)
                                int16_t yawDelta = rvcYaw - prevYaw;
                                if(yawDelta > 18000) yawDelta -= 36000;      // Wrapped from +180 to -180
                                else if(yawDelta < -18000) yawDelta += 36000; // Wrapped from -180 to +180
                                
                                // Ring Buffer Moving Average (updates EVERY packet = 100Hz)
                                // Remove oldest value from sum, add newest value
                                rollingSum -= yawDeltas[ringHead];
                                rollingSum += yawDelta;
                                yawDeltas[ringHead] = yawDelta;
                                
                                // Advance ring buffer position (circular)
                                ringHead++;
                                if(ringHead >= 20) ringHead = 0;
                                
                                // Calculate gyroZ from rolling sum (updates every packet!)
                                // Formula: gyroZ = (avg_delta_per_packet) × (packets_per_sec) × (degrees_per_count)
                                // gyroZ = (rollingSum/20) × 100Hz × 0.01°/count = rollingSum × 0.05 (°/s)
                                gyroZ = (double)rollingSum * 0.05;
                                
                                prevYaw = rvcYaw;
                            }
                        }  // End of "new packet" check

                        // Update watchdog (even for duplicate packets)
                        lastRvcTime = millis();
                        rvcPacketCount++;
                    }
                    else
                    {
                        // Checksum failed
                        checksumErrors++;
                        if(checksumErrors % 100 == 0) {
                            Serial.print("RVC checksum errors: ");
                            Serial.println(checksumErrors);
                        }
                    }

                    // Reset state machine immediately for next packet
                    state = SYNC1;
                    bufferIndex = 0;
                }
                break;
        }
    }
}


void imuHandler()
{
  int16_t temp = 0;

  if (useBNO08x)
  {
      //BNO is reading in its own timer    
      // Fill rest of Panda Sentence - Heading
      temp = yaw;
      itoa(temp, imuHeading, 10);

      // the pitch x10
      temp = (int16_t)pitch;
      itoa(temp, imuPitch, 10);

      // the roll x10
      temp = (int16_t)roll;
      itoa(temp, imuRoll, 10);

      // Debug: show what's being sent to AgOpenGPS
      // static uint32_t lastImuPrint = 0;
      // static uint32_t imuCount = 0;
      // static uint32_t lastRateCalc = 0;
      // 
      // imuCount++;
      // if (millis() - lastRateCalc >= 1000) {
      //     Serial.print("NMEA rate: "); Serial.print(imuCount); Serial.println(" Hz");
      //     imuCount = 0;
      //     lastRateCalc = millis();
      // }
      // 
      // if (millis() - lastImuPrint > 500) {
      //     lastImuPrint = millis();
      //     Serial.print("NMEA -> H:"); Serial.print(imuHeading);
      //     Serial.print(" P:"); Serial.print(imuPitch);
      //     Serial.print(" R:"); Serial.println(imuRoll);
      // }

      // YawRate - calculated from yaw deltas in RVC mode
      if (useYawRate) {
          // gyroZ is calculated in readBNO() using 20-sample moving average (°/s)
          temp = (int16_t)gyroZ;
          itoa(temp, imuYawRate, 10);
      } else {
          itoa(0, imuYawRate, 10);
      }
  }

  // No else, because we want to use dual heading and IMU roll when both connected
  // We have a IMU so apply the dual/IMU roll/heading error to the IMU data.
  if ( useBNO08x && baseLineCheck)
  {
      float dualTemp;   //To convert IMU data (x10) to a float for the PAOGI so we have the decamal point
              
      // the IMU heading raw
      // dualTemp = yaw * 0.1;
      // dtostrf(dualTemp, 3, 1, imuHeading);          

      // the IMU heading fused to the dual heading
      fuseIMU();
      dtostrf(imuCorrected, 3, 1, imuHeading);
    
      // the pitch
      dualTemp = (int16_t)pitch * 0.1;
      dtostrf(dualTemp, 3, 1, imuPitch);

      // the roll
      dualTemp = (int16_t)roll * 0.1;
      //If dual heading correction is 90deg (antennas left/right) correct the IMU roll
      if(headingcorr == 900)
      {
        dualTemp += rollDeltaSmooth;
      }
      dtostrf(dualTemp, 3, 1, imuRoll);

  }
  else if (!useBNO08x)  //Not using IMU so put dual Heading & Roll in direct.
  {
      // the roll
      if (makeOGI)
      {
        dtostrf(rollDual, 4, 2, imuRoll);
      }
      else
      {
        itoa(rollDual * 10, imuRoll, 10);
      }

      // the Dual heading raw
      if (makeOGI)
      {
        dtostrf(heading, 4, 2, imuHeading);
      }
      else
      {
        itoa(heading * 10, imuHeading, 10);
      }

      // the pitch
      dtostrf(pitchDual, 4, 4, imuPitch);
  }
}

void imuDualDelta()
{
                                       //correctionHeading is IMU heading in radians
   gpsHeading = heading * DEG_TO_RAD;  //gpsHeading is Dual heading in radians

   //Difference between the IMU heading and the GPS heading
   gyroDelta = (correctionHeading + imuGPS_Offset) - gpsHeading;
   if (gyroDelta < 0) gyroDelta += twoPI;

   //calculate delta based on circular data problem 0 to 360 to 0, clamp to +- 2 Pi
   if (gyroDelta >= -PIBy2 && gyroDelta <= PIBy2) gyroDelta *= -1.0;
   else
   {
       if (gyroDelta > PIBy2) { gyroDelta = twoPI - gyroDelta; }
       else { gyroDelta = (twoPI + gyroDelta) * -1.0; }
   }
   if (gyroDelta > twoPI) gyroDelta -= twoPI;
   if (gyroDelta < -twoPI) gyroDelta += twoPI;

   //if the gyro and last corrected fix is < 10 degrees, super low pass for gps
   if (abs(gyroDelta) < 0.18)
   {
       //a bit of delta and add to correction to current gyro
       imuGPS_Offset += (gyroDelta * (0.1));
       if (imuGPS_Offset > twoPI) imuGPS_Offset -= twoPI;
       if (imuGPS_Offset < -twoPI) imuGPS_Offset += twoPI;
   }
   else
   {
       //a bit of delta and add to correction to current gyro
       imuGPS_Offset += (gyroDelta * (0.2));
       if (imuGPS_Offset > twoPI) imuGPS_Offset -= twoPI;
       if (imuGPS_Offset < -twoPI) imuGPS_Offset += twoPI;
   }

   //So here how we have the difference between the IMU heading and the Dual GPS heading
   //This "imuGPS_Offset" will be used in imuHandler() when the GGA arrives 

   //Calculate the diffrence between dual and imu roll
   float imuRoll;
   imuRoll = (int16_t)roll * 0.1;
   rollDelta = rollDual - imuRoll;
   rollDeltaSmooth = (rollDeltaSmooth * 0.7) + (rollDelta * 0.3);
}

void fuseIMU()
{     
   //determine the Corrected heading based on gyro and GPS
   imuCorrected = correctionHeading + imuGPS_Offset;
   if (imuCorrected > twoPI) imuCorrected -= twoPI;
   if (imuCorrected < 0) imuCorrected += twoPI;

   imuCorrected = imuCorrected * RAD_TO_DEG; 
}

void BuildNmea(void)
{
    strcpy(nmea, "");

    if (makeOGI) strcat(nmea, "$PAOGI,");
    else strcat(nmea, "$PANDA,");

    strcat(nmea, fixTime);
    strcat(nmea, ",");

    strcat(nmea, latitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, longitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");

    // 6
    strcat(nmea, fixQuality);
    strcat(nmea, ",");

    strcat(nmea, numSats);
    strcat(nmea, ",");

    strcat(nmea, HDOP);
    strcat(nmea, ",");

    strcat(nmea, altitude);
    strcat(nmea, ",");

    //10
    strcat(nmea, ageDGPS);
    strcat(nmea, ",");

    //11
    strcat(nmea, speedKnots);
    strcat(nmea, ",");

    //12
    strcat(nmea, imuHeading);
    strcat(nmea, ",");

    //13
    strcat(nmea, imuRoll);
    strcat(nmea, ",");

    //14
    strcat(nmea, imuPitch);
    strcat(nmea, ",");

    //15
    strcat(nmea, imuYawRate);

    strcat(nmea, "*");

    CalculateChecksum();

    strcat(nmea, "\r\n");
    
    // Debug: Print complete NMEA sentence occasionally
    // static uint32_t nmeaCount = 0;
    // if (++nmeaCount % 10 == 0) {  // Print every 10th NMEA
    //     Serial.print("NMEA: ");
    //     Serial.println(nmea);
    // }

    if (sendUSB) { SerialAOG.write(nmea); } // Send USB GPS data if enabled in user settings
    
    if (Ethernet_running)   //If ethernet running send the GPS there
    {
        int len = strlen(nmea);
        
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write((const uint8_t*)nmea, len);
        Eth_udpPAOGI.endPacket();
    }
}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex2);
}

/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with


  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV

  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
      6A          The checksum data, always begins with

  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
     48          Checksum
*/
