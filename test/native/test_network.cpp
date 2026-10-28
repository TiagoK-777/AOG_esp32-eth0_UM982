#include <Arduino.h>
#include <WiFiUdp.h>
#include <cassert>
#include <cstring>
#include <deque>
#include <iostream>
#include "config/defines.h"
#include "network/ethernet.h"
#include "network/pgn_protocol.h"
#include "network/udp_rx.h"
#include "network/udp_tx.h"
#include "state/imu_state.h"
#include "state/steer_state.h"

const UserSettings userSettings{};
namespace {
std::deque<udp_tx::Packet> queue;
// Pacote literal de SourceCode/AgIO/Source/Forms/UDP.designer.cs.
const std::vector<uint8_t> agioHello{0x80,0x81,0x7F,200,3,56,0,0,0x47};

void receive(const std::vector<uint8_t>& bytes) {
    fakeUdpTx.clear();
    fakeUdpRx[PORT_AUTOSTEER].push_back(bytes);
    network::poll();
}
void checkReply(size_t index, uint8_t id) {
    const auto& packet = fakeUdpTx.at(index);
    assert(packet.sourcePort == PORT_AUTOSTEER && packet.destinationPort == PORT_DESTINATION);
    assert(packet.destination[0] == 192 && packet.destination[1] == 168 &&
           packet.destination[2] == 5 && packet.destination[3] == 255);
    const auto& bytes = packet.bytes;
    assert(bytes.size() == 11 && bytes[0] == 0x80 && bytes[1] == 0x81);
    assert(bytes[2] == id && bytes[3] == id && bytes[4] == 5);
    uint8_t sum = 0;
    for (size_t i = 2; i < 10; ++i) sum += bytes[i];
    assert(bytes[10] == sum);
}
}
namespace ethernet {
IPAddress destination() { return IPAddress(192,168,5,255); }
IPAddress localAddress() { return IPAddress(192,168,5,126); }
}
namespace udp_tx {
bool init() { queue.clear(); return true; }
bool enqueueTo(Channel channel, const uint8_t* bytes, size_t length, IPAddress destination, uint16_t port) {
    assert(length <= UDP_TX_MAX_PAYLOAD);
    Packet packet{};
    packet.channel = channel; packet.length = length; packet.port = port;
    for (size_t i = 0; i < 4; ++i) packet.destination[i] = destination[i];
    std::memcpy(packet.payload, bytes, length);
    queue.push_back(packet);
    return true;
}
bool enqueue(Channel channel, const uint8_t* bytes, size_t length) {
    return enqueueTo(channel, bytes, length, ethernet::destination(), PORT_DESTINATION);
}
bool take(Packet& packet) {
    if (queue.empty()) return false;
    packet = queue.front(); queue.pop_front(); return true;
}
}

int main() {
    assert(network::setup());
    receive(agioHello);
    assert(fakeUdpTx.empty());

    SteerStatus steer;
    steer.available = true;
    steer.steerAngleActual = -12.34f;
    steer.helloSteerPosition = -300;
    steer.switchByte = 5;
    state::publishSteer(steer);
    receive(agioHello);
    assert(fakeUdpTx.size() == 1);
    checkReply(0, 126);
    const auto& reply = fakeUdpTx[0].bytes;
    assert(reply[5] == 0x2E && reply[6] == 0xFB);
    assert(reply[7] == 0xD4 && reply[8] == 0xFE && reply[9] == 5);
    // Hello indica módulo disponível, inclusive com piloto desengajado.
    assert(!steer.engaged);

    ImuState imu;
    imu.useBNO08x = true;
    state::publishImu(imu);
    receive(agioHello);
    assert(fakeUdpTx.size() == 2);
    checkReply(0, 126); checkReply(1, 121);
    steer.available = false;
    state::publishSteer(steer);
    receive(agioHello);
    assert(fakeUdpTx.size() == 1);
    checkReply(0, 121);

    auto standardHello = agioHello;
    pgn::checksum(standardHello.data(), standardHello.size());
    assert(standardHello.back() == 0x82);
    receive(standardHello);
    assert(fakeUdpTx.size() == 1);
    checkReply(0, 121);

    // A compatibilidade não aceita alterações, truncamento ou dados extras.
    uint32_t rejected = network::stats().invalidPackets;
    for (size_t i = 0; i < agioHello.size(); ++i) {
        auto corrupt = agioHello;
        corrupt[i] ^= 1;
        receive(corrupt);
        assert(fakeUdpTx.empty());
        assert(network::stats().invalidPackets == ++rejected);
    }
    for (size_t count = 1; count < agioHello.size(); ++count) {
        receive(std::vector<uint8_t>(agioHello.begin(), agioHello.begin() + count));
        assert(fakeUdpTx.empty());
        assert(network::stats().invalidPackets == ++rejected);
    }
    auto extra = agioHello; extra.push_back(0);
    receive(extra);
    assert(fakeUdpTx.empty());
    assert(network::stats().invalidPackets == ++rejected);

    // Um comando com checksum incorreto nunca publica guidance.
    steer.available = true; state::publishSteer(steer);
    std::vector<uint8_t> command{0x80,0x81,0x7F,254,8,123,0,1,0x2E,0xFB,3,4,5,0};
    pgn::checksum(command.data(), command.size());
    const auto sequence = state::guidanceSnapshot().sequence;
    auto corrupt = command; corrupt.back() ^= 1;
    receive(corrupt);
    assert(fakeUdpTx.empty() && state::guidanceSnapshot().sequence == sequence);
    receive(command);
    assert(fakeUdpTx.size() == 1 && fakeUdpTx[0].bytes[3] == 253);
    assert(state::guidanceSnapshot().sequence != sequence);
    std::cout << "Rede: hello AgIO, respostas e rejeição de pacotes inválidos OK\n";
}
