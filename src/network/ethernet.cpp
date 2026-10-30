/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/


#include <Arduino.h>
#include <ETH.h>
#include "network/ethernet.h"
#include "config/settings.h"
#include "diagnostics/log.h"
namespace {
IPAddress broadcast;
IPAddress local;
}
namespace ethernet {
bool setup(bool autosteerAvailable) {
    AOG_LOG_PRINTLN("Initializing ESP32 Ethernet with static IP address");
    if (!ETH.begin()) return false;
    ConfigIP subnet = config::snapshot().networkAddress;
    local = IPAddress(subnet.ipOne, subnet.ipTwo, subnet.ipThree, autosteerAvailable ? 126 : 120);
    broadcast = IPAddress(subnet.ipOne, subnet.ipTwo, subnet.ipThree, 255);
    if (!ETH.config(local, IPAddress(subnet.ipOne, subnet.ipTwo, subnet.ipThree, 1), IPAddress(255,255,255,0))) return false;
    AOG_LOG_PRINTLN("Ethernet status OK");
    AOG_LOG_PRINT("IP set Manually: "); AOG_LOG_PRINTLN(local);
    return true;
}
IPAddress destination() { return broadcast; }
IPAddress localAddress() { return local; }
}
