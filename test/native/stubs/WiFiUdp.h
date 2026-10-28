#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>
#include <IPAddress.h>

struct FakeUdpPacket {
    uint16_t sourcePort, destinationPort;
    IPAddress destination;
    std::vector<uint8_t> bytes;
};
inline std::map<uint16_t, std::deque<std::vector<uint8_t>>> fakeUdpRx;
inline std::vector<FakeUdpPacket> fakeUdpTx;

class WiFiUDP {
public:
    uint8_t begin(uint16_t port) { port_ = port; return 1; }
    int parsePacket() { return fakeUdpRx[port_].empty() ? 0 : fakeUdpRx[port_].front().size(); }
    int read(uint8_t* output, size_t capacity) {
        auto& packets = fakeUdpRx[port_];
        if (packets.empty()) return 0;
        const auto& bytes = packets.front();
        size_t count = std::min(capacity, bytes.size());
        std::copy_n(bytes.begin(), count, output);
        packets.pop_front();
        return count;
    }
    IPAddress remoteIP() { return IPAddress(192,168,5,10); }
    int beginPacket(IPAddress destination, uint16_t port) {
        outgoing_ = FakeUdpPacket{port_, port, destination, {}};
        return 1;
    }
    size_t write(const uint8_t* bytes, size_t count) {
        outgoing_.bytes.insert(outgoing_.bytes.end(), bytes, bytes + count);
        return count;
    }
    int endPacket() { fakeUdpTx.push_back(outgoing_); return 1; }
private:
    uint16_t port_ = 0;
    FakeUdpPacket outgoing_{};
};
