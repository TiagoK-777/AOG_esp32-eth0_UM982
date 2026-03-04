/*
   UDP NTRIP passthrough for ESP32 WT32-ETH01
   For AgOpenGPS
   Relies on SerialGPS TX buffer (2048 bytes, set in setup) so that
   write() copies to RAM and returns in microseconds. The UART ISR
   drains the buffer to hardware FIFO in background.
*/

#include <Arduino.h>
#include "GlobalVariables.h"

void udpNtrip()
{
  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (!Ethernet_running)
  {
    return;
  }

  unsigned int packetLength = Eth_udpNtrip.parsePacket();
  if (packetLength > 0)
  {
    if (packetLength > sizeof(Eth_NTRIP_packetBuffer))
    {
      packetLength = sizeof(Eth_NTRIP_packetBuffer);
    }
    Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLength);
    SerialGPS->write(reinterpret_cast<const uint8_t*>(Eth_NTRIP_packetBuffer), packetLength);
  }
}
