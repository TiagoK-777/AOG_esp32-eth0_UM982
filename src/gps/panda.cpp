/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
*/


#include <Arduino.h>
#include <cstdio>
#include "gps/context.h"
#include "config/settings.h"
#include "network/udp_tx.h"
namespace gps {
void BuildNmea() {
    char nmea[150];
    const Context& c = context;
    int length = snprintf(nmea, sizeof(nmea),
        "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s*",
        userSettings.makeOGI ? "$PAOGI" : "$PANDA", c.fixTime,
        c.latitude, c.latNS, c.longitude, c.lonEW, c.fixQuality,
        c.numSats, c.HDOP, c.altitude, c.ageDGPS, c.speedKnots,
        c.imuHeading, c.imuRoll, c.imuPitch, c.imuYawRate);
    if (length < 0 || static_cast<size_t>(length) + 4 >= sizeof(nmea)) return;
    uint8_t sum = 0;
    for (int i = 1; i < length - 1; ++i) sum ^= static_cast<uint8_t>(nmea[i]);
    snprintf(nmea + length, sizeof(nmea) - length, "%02X\r\n", sum);
    length += 4;
    if (userSettings.sendUSB) Serial.write(reinterpret_cast<const uint8_t*>(nmea), length);
    udp_tx::enqueue(udp_tx::Channel::GPS, reinterpret_cast<const uint8_t*>(nmea), length);
}
}
