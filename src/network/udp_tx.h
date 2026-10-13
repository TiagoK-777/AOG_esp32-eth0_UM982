#pragma once
#include <cstddef>
#include <cstdint>
#include <IPAddress.h>
namespace udp_tx {
enum class Channel : uint8_t { Autosteer, GPS };
struct Packet {
    Channel channel;
    uint8_t destination[4];
    uint16_t port;
    uint16_t length;
    uint8_t payload[256];
};
bool init();
bool enqueue(Channel channel, const uint8_t* data, size_t length);
bool enqueueTo(Channel channel, const uint8_t* data, size_t length, IPAddress destination, uint16_t port);
bool take(Packet& packet);
uint32_t droppedPackets();
}
