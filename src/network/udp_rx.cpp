/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/


#include <Arduino.h>
#include <WiFiUdp.h>
#include "network/udp_rx.h"
#include "network/udp_tx.h"
#include "network/ethernet.h"
#include "network/rtcm_buffer.h"
#include "network/pgn_protocol.h"
#include "config/defines.h"
#include "state/guidance_state.h"
#include "state/steer_state.h"
#include "state/imu_state.h"
#include "state/pending.h"
#include "state/shared_value.h"
namespace {
WiFiUDP steerSocket, ntripSocket, gpsSocket;
uint8_t receiveBuffer[1460];
uint8_t sensorCount = 0;
network::Stats counters{};
SharedValue<network::Stats> publishedCounters;

void sendStatus() {
    SteerStatus status = state::steerSnapshot();
    uint8_t reply[] = {0x80,0x81,126,253,8,0,0,0,0,0,0,status.switchByte,status.pwmDisplay,0};
    int16_t angle = static_cast<int16_t>(status.steerAngleActual * 100);
    reply[5] = angle; reply[6] = angle >> 8;
    reply[7] = static_cast<uint8_t>(9999); reply[8] = 9999 >> 8;
    reply[9] = static_cast<uint8_t>(8888); reply[10] = 8888 >> 8;
    pgn::checksum(reply, sizeof(reply));
    udp_tx::enqueue(udp_tx::Channel::Autosteer, reply, sizeof(reply));
    if (config::snapshot().steerConfig.CurrentSensor && sensorCount++ > 2) {
        uint8_t sensor[] = {0x80,0x81,126,250,8,static_cast<uint8_t>(status.sensorReading),0,0,0,0,0,0,0,0};
        pgn::checksum(sensor, sizeof(sensor));
        udp_tx::enqueue(udp_tx::Channel::Autosteer, sensor, sizeof(sensor));
        sensorCount = 0;
    }
}
void dispatch(const uint8_t* d, size_t length, IPAddress remote) {
    if (!pgn::valid(d, length)) { ++counters.invalidPackets; return; }
    SteerStatus status = state::steerSnapshot();
    switch (d[3]) {
    case 254: {
        GuidanceCommand command;
        if (!status.available || !pgn::decodeGuidance(d, length, command)) break;
        command.receivedMs = millis();
        state::publishGuidance(command);
        sendStatus();
        break;
    }
    case 252: {
        Storage settings;
        if (status.available && pgn::decodeSettings(d, length, settings)) pending::stageSettings(settings);
        else ++counters.invalidPackets;
        break;
    }
    case 251: {
        Setup settings;
        if (pgn::decodeConfig(d, length, settings)) pending::stageConfig(settings);
        else ++counters.invalidPackets;
        break;
    }
    case 200: {
        if (status.available) {
            uint8_t hello[] = {0x80,0x81,126,126,5,0,0,0,0,status.switchByte,71};
            int16_t angle = static_cast<int16_t>(status.steerAngleActual * 100);
            hello[5] = angle; hello[6] = angle >> 8;
            hello[7] = status.helloSteerPosition; hello[8] = status.helloSteerPosition >> 8;
            pgn::checksum(hello, sizeof(hello));
            udp_tx::enqueue(udp_tx::Channel::Autosteer, hello, sizeof(hello));
        }
        if (state::imuSnapshot().useBNO08x) {
            uint8_t hello[] = {128,129,121,121,5,0,0,0,0,0,0};
            pgn::checksum(hello, sizeof(hello));
            udp_tx::enqueue(udp_tx::Channel::Autosteer, hello, sizeof(hello));
        }
        break;
    }
    case 201:
        if (d[4] == 5 && d[5] == 201 && d[6] == 201)
            pending::stageNetwork(ConfigIP{d[7],d[8],d[9]});
        break;
    case 202:
        if (d[4] == 3 && d[5] == 202 && d[6] == 202) {
            IPAddress local = ethernet::localAddress();
            uint8_t reply[] = {128,129,local[3],203,7,local[0],local[1],local[2],local[3],remote[0],remote[1],remote[2],0};
            pgn::checksum(reply, sizeof(reply));
            udp_tx::enqueueTo(udp_tx::Channel::Autosteer, reply, sizeof(reply), IPAddress(255,255,255,255), PORT_DESTINATION);
        }
        break;
    }
}
}
namespace network {
bool setup() {
    bool steer = steerSocket.begin(PORT_AUTOSTEER);
    bool ntrip = ntripSocket.begin(PORT_NTRIP);
    bool gps = gpsSocket.begin(PORT_GPS);
    return steer && ntrip && gps && udp_tx::init();
}
void poll() {
    int length = steerSocket.parsePacket();
    if (length > 0) {
        int count = steerSocket.read(receiveBuffer, sizeof(receiveBuffer));
        if (count == length) dispatch(receiveBuffer, count, steerSocket.remoteIP());
        else ++counters.invalidPackets;
    }
    length = ntripSocket.parsePacket();
    if (length > 0) {
        int count = ntripSocket.read(receiveBuffer, sizeof(receiveBuffer));
        if (count == length) rtcm::enqueue(receiveBuffer, count);
        else ++counters.invalidPackets;
    }
    // RX e TX têm orçamento por passagem para não monopolizar a task.
    for (uint8_t i = 0; i < 2; ++i) {
        udp_tx::Packet packet;
        if (!udp_tx::take(packet)) break;
        WiFiUDP& socket = packet.channel == udp_tx::Channel::GPS ? gpsSocket : steerSocket;
        IPAddress destination(packet.destination[0], packet.destination[1], packet.destination[2], packet.destination[3]);
        bool ok = socket.beginPacket(destination, packet.port);
        if (ok) ok = socket.write(packet.payload, packet.length) == packet.length;
        if (ok) ok = socket.endPacket() == 1;
        if (!ok) ++counters.sendFailures;
    }
    publishedCounters.publish(counters);
}
Stats stats() { return publishedCounters.snapshot(); }
}
