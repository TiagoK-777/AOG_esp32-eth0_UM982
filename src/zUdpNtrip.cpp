/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
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
    Serial.print("[NTRIP] Recebido ");
    Serial.print(packetLength);
    Serial.print(" bytes -> Enviando para SerialGPS (GPIO17 TX)...");
    
    Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLength);
    size_t bytesWritten = SerialGPS->write(Eth_NTRIP_packetBuffer, packetLength);
    
    Serial.print(" Enviado: ");
    Serial.print(bytesWritten);
    Serial.println(" bytes");
  }
}
