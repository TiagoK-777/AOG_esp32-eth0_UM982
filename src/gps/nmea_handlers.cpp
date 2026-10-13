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
  context.dualReadyRelPos = true;
  //digitalWrite(GPSRED_LED, LOW);   //Turn red GPS LED OFF (we are now in dual mode so green LED) - Commented to save ESP32 pins

  // HPR Heading
  parser.getArg(1, context.umHeading);
  context.heading = atof(context.umHeading);
  if ( userSettings.filterHeading )
    {
      float tempHeading;
      tempHeading = headingFilter.updateEstimate(context.heading);
      context.heading = tempHeading;
    }

  // HPR Substitute imu.pitch for imu.roll
  if ( parser.getArg(2, context.umRoll) )
  {
    context.rollDual = atof(context.umRoll);
    //digitalWrite(GPSGREEN_LED, HIGH);   //Turn green GPS LED ON - Commented to save ESP32 pins
    if ( userSettings.filterRoll )
      {
        float tempRoll;
        tempRoll = rollFilter.updateEstimate(context.rollDual);
        context.rollDual = tempRoll;
      }
  }
  else
  {
    //digitalWrite(GPSGREEN_LED, context.blink);  //Flash the green GPS LED - Commented to save ESP32 pins
  }

  // Solution quality factor
  parser.getArg(4, context.solQuality);

  if (context.solQuality >= 4)
    {
       if (imu.useBNO08x)
       {
           if (userSettings.baseLineCheck)
           {
               imuDualDelta();         //Find the error between latest IMU reading and this dual message
              //context.dualReadyRelPos = false;  //RelPos ready is false because we just saved the error for running from the IMU
           }

       }
       else
       {
           imuHandler();             //No IMU so use dual data direct
           context.dualReadyRelPos = true;   //RelPos ready is true so PAOGI will send when the GGA is also ready
       }
    }
}
}
