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
    double heading = context.heading;
    double roll = context.rollDual;
    double pitch = context.pitchDual;
    double yawRate = 0;
    if (imu.useBNO08x)
    {
        heading = imu.yaw * 0.1;
        roll = imu.roll * 0.1;
        pitch = imu.pitch * 0.1;
        if (userSettings.useYawRate) yawRate = imu.gyroZ;
        if (userSettings.baseLineCheck)
        {
            fuseIMU();
            heading = context.imuCorrected;
            if (userSettings.headingcorr == 900) roll += context.rollDeltaSmooth;
            else if (userSettings.headingcorr == 0) pitch += context.pitchDeltaSmooth;
        }
    }

    if (userSettings.makeOGI)
    {
        dtostrf(heading, 3, imu.useBNO08x ? 1 : 2, context.imuHeading);
        dtostrf(roll, 3, imu.useBNO08x ? 1 : 2, context.imuRoll);
        dtostrf(pitch, 3, imu.useBNO08x ? 1 : 4, context.imuPitch);
        dtostrf(yawRate, 3, 1, context.imuYawRate);
    }
    else
    {
        itoa(lround(heading * 10), context.imuHeading, 10);
        itoa(lround(roll * 10), context.imuRoll, 10);
        itoa(lround(pitch * 10), context.imuPitch, 10);
        itoa(lround(yawRate * 10), context.imuYawRate, 10);
    }
}

void imuDualDelta()
{
    context.gpsHeading = context.heading * DEG_TO_RAD;
    const double error = context.gpsHeading - (imu.correctionHeading + context.imuGPS_Offset);
    // Menor diferença circular, inclusive erros entre 90 e 180 graus.
    context.gyroDelta = atan2(sin(error), cos(error));
    const double gain = abs(context.gyroDelta) < 0.18 ? 0.1 : 0.2;
    context.imuGPS_Offset = fmod(context.imuGPS_Offset + context.gyroDelta * gain, twoPI);

    // A baseline mede apenas a inclinação do eixo em que foi montada.
    if (userSettings.headingcorr == 900)
    {
        context.rollDelta = context.rollDual - imu.roll * 0.1;
        context.rollDeltaSmooth = context.rollDeltaSmooth * 0.7 + context.rollDelta * 0.3;
    }
    else if (userSettings.headingcorr == 0)
    {
        context.pitchDelta = context.pitchDual - imu.pitch * 0.1;
        context.pitchDeltaSmooth = context.pitchDeltaSmooth * 0.7 + context.pitchDelta * 0.3;
    }
}

void fuseIMU()
{
    context.imuCorrected = fmod(imu.correctionHeading + context.imuGPS_Offset, twoPI);
    if (context.imuCorrected < 0) context.imuCorrected += twoPI;
    context.imuCorrected *= RAD_TO_DEG;
}
}
