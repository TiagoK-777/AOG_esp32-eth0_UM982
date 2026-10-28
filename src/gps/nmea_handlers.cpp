/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <cmath>
#include <cstdlib>
#include "gps/context.h"
#include "config/settings.h"
#include "config/defines.h"
namespace gps {
void errorHandler()
{
  // Error handling can be added here if needed
}

void GGA_Handler() //Rec'd GGA
{
    // fix time
    parser.getArg(0, context.fixTime);

    // context.latitude
    parser.getArg(1, context.latitude);
    parser.getArg(2, context.latNS);

    // context.longitude
    parser.getArg(3, context.longitude);
    parser.getArg(4, context.lonEW);

    // fix quality
    parser.getArg(5, context.fixQuality);

    // satellite #
    parser.getArg(6, context.numSats);

    // context.HDOP
    parser.getArg(7, context.HDOP);

    // context.altitude
    parser.getArg(8, context.altitude);

    // time of last DGPS update
    parser.getArg(12, context.ageDGPS);

    if (context.blink)
    {
        //digitalWrite(GGAReceivedLED, HIGH);  //Commented to save ESP32 pins
    }
    else
    {
        //digitalWrite(GGAReceivedLED, LOW);   //Commented to save ESP32 pins
    }

    context.blink = !context.blink;
    context.GGA_Available = true;

    context.dualReadyGGA = true;

    context.gpsReadyTime = millis();    //Used for GGA timeout (LED's ETC)
}

void VTG_Handler()
{
  // vtg context.heading
  parser.getArg(0, context.vtgHeading);

  // vtg Speed knots
  parser.getArg(4, context.speedKnots);
}

void HPR_Handler()
{
    char headingText[sizeof(context.umHeading)]{};
    char tiltText[sizeof(context.umRoll)]{};
    char qualityText[4]{};
    // Não publicar parcialmente uma época nem reutilizar qualidade anterior.
    context.dualReadyRelPos = false;
    if (!parser.getArg(1, headingText) || !parser.getArg(2, tiltText) ||
        !parser.getArg(4, qualityText) || !headingText[0] || !tiltText[0] || !qualityText[0]) return;
    char* end = nullptr;
    double heading = strtod(headingText, &end);
    if (*end || !std::isfinite(heading) || heading < 0 || heading > 360) return;
    double tilt = strtod(tiltText, &end);
    if (*end || !std::isfinite(tilt) || abs(tilt) > 180) return;
    long quality = strtol(qualityText, &end, 10);
    if (*end || quality < 0 || quality > 9) return;

    memcpy(context.umHeading, headingText, sizeof(headingText));
    memcpy(context.umRoll, tiltText, sizeof(tiltText));
    context.heading = userSettings.filterHeading ? headingFilter.updateEstimate(heading) : heading;
    if (userSettings.filterRoll) tilt = rollFilter.updateEstimate(tilt);
    context.rollDual = userSettings.headingcorr == 900 ? tilt : 0;
    context.pitchDual = userSettings.headingcorr == 0 ? tilt : 0;
    context.solQuality = static_cast<int>(quality);
    // Uma época de qualidade baixa permite holdover, mas não recalibra a IMU.
    context.dualReadyRelPos = true;
    if (quality >= 4 && imu.useBNO08x && userSettings.baseLineCheck) imuDualDelta();
}
}
