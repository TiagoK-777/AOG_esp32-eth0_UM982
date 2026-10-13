/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include "gps/context.h"
#include "config/settings.h"
#include "config/defines.h"
namespace gps {
void imuHandler()
{
  int16_t temp = 0;

  if (imu.useBNO08x)
  {
      if (userSettings.makeOGI)
      {
          // PAOGI: valores em graus decimais (float)
          // yaw, pitch, roll estão em unidades de 0.1° (x10), converter para graus
          dtostrf(imu.yaw * 0.1, 3, 1, context.imuHeading);    // Ex: "123.4"
          dtostrf(imu.pitch * 0.1, 3, 1, context.imuPitch);     // Ex: "0.5"
          dtostrf(imu.roll * 0.1, 3, 1, context.imuRoll);       // Ex: "-1.2"

          // YawRate in °/s
          if (userSettings.useYawRate) {
              dtostrf(imu.gyroZ, 3, 1, context.imuYawRate);      // Ex: "5.2"
          } else {
              dtostrf(0.0, 3, 1, context.imuYawRate);
          }
      }
      else
      {
          // PANDA: valores como inteiros x10
          temp = imu.yaw;
          itoa(temp, context.imuHeading, 10);

          temp = (int16_t)imu.pitch;
          itoa(temp, context.imuPitch, 10);

          temp = (int16_t)imu.roll;
          itoa(temp, context.imuRoll, 10);

          // YawRate as integer x10 (0.1°/s units, matching Teensy format)
          if (userSettings.useYawRate) {
              temp = (int16_t)(imu.gyroZ * 10.0);
              itoa(temp, context.imuYawRate, 10);
          } else {
              itoa(0, context.imuYawRate, 10);
          }
      }
  }

  // No else, because we want to use dual heading and IMU roll when both connected
  // We have a IMU so apply the dual/IMU roll/heading error to the IMU data.
  if ( imu.useBNO08x && userSettings.baseLineCheck)
  {
      float dualTemp;

      // the IMU heading fused to the dual heading
      fuseIMU();
      dtostrf(context.imuCorrected, 3, 1, context.imuHeading);

      // the pitch
      dualTemp = (int16_t)imu.pitch * 0.1;
      dtostrf(dualTemp, 3, 1, context.imuPitch);

      // the roll
      dualTemp = (int16_t)imu.roll * 0.1;
      //If dual heading correction is 90deg (antennas left/right) correct the IMU roll
      if(userSettings.headingcorr == 900)
      {
        dualTemp += context.rollDeltaSmooth;
      }
      dtostrf(dualTemp, 3, 1, context.imuRoll);

      // YawRate for dual+IMU mode
      if (userSettings.useYawRate) {
          dtostrf(imu.gyroZ, 3, 1, context.imuYawRate);
      } else {
          dtostrf(0.0, 3, 1, context.imuYawRate);
      }
  }
  else if (!imu.useBNO08x)  //Not using IMU so put dual Heading & Roll in direct.
  {
      // the roll
      if (userSettings.makeOGI)
      {
        dtostrf(context.rollDual, 4, 2, context.imuRoll);
      }
      else
      {
        itoa(context.rollDual * 10, context.imuRoll, 10);
      }

      // the Dual heading raw
      if (userSettings.makeOGI)
      {
        dtostrf(context.heading, 4, 2, context.imuHeading);
      }
      else
      {
        itoa(context.heading * 10, context.imuHeading, 10);
      }

      // the pitch
      dtostrf(context.pitchDual, 4, 4, context.imuPitch);

      // YawRate not available without IMU
      if (userSettings.makeOGI) {
          dtostrf(0.0, 3, 1, context.imuYawRate);
      } else {
          itoa(0, context.imuYawRate, 10);
      }
  }
}

void imuDualDelta()
{
                                       //correctionHeading is IMU heading in radians
   context.gpsHeading = context.heading * DEG_TO_RAD;  //gpsHeading is Dual heading in radians

   //Difference between the IMU heading and the GPS heading
   context.gyroDelta = (imu.correctionHeading + context.imuGPS_Offset) - context.gpsHeading;
   if (context.gyroDelta < 0) context.gyroDelta += twoPI;

   //calculate delta based on circular data problem 0 to 360 to 0, clamp to +- 2 Pi
   if (context.gyroDelta >= -PIBy2 && context.gyroDelta <= PIBy2) context.gyroDelta *= -1.0;
   else
   {
       if (context.gyroDelta > PIBy2) { context.gyroDelta = twoPI - context.gyroDelta; }
       else { context.gyroDelta = (twoPI + context.gyroDelta) * -1.0; }
   }
   if (context.gyroDelta > twoPI) context.gyroDelta -= twoPI;
   if (context.gyroDelta < -twoPI) context.gyroDelta += twoPI;

   //if the gyro and last corrected fix is < 10 degrees, super low pass for gps
   if (abs(context.gyroDelta) < 0.18)
   {
       //a bit of delta and add to correction to current gyro
       context.imuGPS_Offset += (context.gyroDelta * (0.1));
       if (context.imuGPS_Offset > twoPI) context.imuGPS_Offset -= twoPI;
       if (context.imuGPS_Offset < -twoPI) context.imuGPS_Offset += twoPI;
   }
   else
   {
       //a bit of delta and add to correction to current gyro
       context.imuGPS_Offset += (context.gyroDelta * (0.2));
       if (context.imuGPS_Offset > twoPI) context.imuGPS_Offset -= twoPI;
       if (context.imuGPS_Offset < -twoPI) context.imuGPS_Offset += twoPI;
   }

   //So here how we have the difference between the IMU heading and the Dual GPS heading
   //This "imuGPS_Offset" will be used in imuHandler() when the GGA arrives

   //Calculate the diffrence between dual and imu roll
   float imuRollDegrees = (int16_t)imu.roll * 0.1;
   context.rollDelta = context.rollDual - imuRollDegrees;
   context.rollDeltaSmooth = (context.rollDeltaSmooth * 0.7) + (context.rollDelta * 0.3);
}

void fuseIMU()
{
   //determine the Corrected heading based on gyro and GPS
   context.imuCorrected = imu.correctionHeading + context.imuGPS_Offset;
   if (context.imuCorrected > twoPI) context.imuCorrected -= twoPI;
   if (context.imuCorrected < 0) context.imuCorrected += twoPI;

   context.imuCorrected = context.imuCorrected * RAD_TO_DEG;
}
}
