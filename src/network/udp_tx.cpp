#include "network/udp_tx.h"
#include "network/ethernet.h"
#include "config/defines.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>
#include <cstring>
#include <type_traits>
namespace {
static_assert(std::is_trivially_copyable<udp_tx::Packet>::value, "A fila UDP deve copiar somente dados simples");
StaticQueue_t queueStorage;
uint8_t packetStorage[UDP_TX_QUEUE_DEPTH * sizeof(udp_tx::Packet)];
QueueHandle_t queue = nullptr;
std::atomic<uint32_t> dropped{0};
}
namespace udp_tx {
bool init() {
    queue = xQueueCreateStatic(UDP_TX_QUEUE_DEPTH, sizeof(Packet), packetStorage, &queueStorage);
    return queue != nullptr;
}
bool enqueueTo(Channel channel, const uint8_t* data, size_t length, IPAddress destination, uint16_t port) {
    if (!queue || !data || length > UDP_TX_MAX_PAYLOAD) { ++dropped; return false; }
    Packet packet{};
    packet.channel = channel;
    for (uint8_t i = 0; i < 4; ++i) packet.destination[i] = destination[i];
    packet.port = port; packet.length = length;
    memcpy(packet.payload, data, length);
    if (xQueueSend(queue, &packet, 0) != pdTRUE) { ++dropped; return false; }
    return true;
}
bool enqueue(Channel channel, const uint8_t* data, size_t length) {
    return enqueueTo(channel, data, length, ethernet::destination(), PORT_DESTINATION);
}
bool take(Packet& packet) { return queue && xQueueReceive(queue, &packet, 0) == pdTRUE; }
uint32_t droppedPackets() { return dropped.load(); }
}
