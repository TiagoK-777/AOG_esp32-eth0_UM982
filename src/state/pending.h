#pragma once
#include "config/settings.h"

namespace pending {
constexpr uint8_t Settings = 1, Config = 2, Network = 4;
struct Update { ConfigSnapshot values; uint8_t mask = 0; };
void stageSettings(const Storage& settings);
void stageConfig(const Setup& config);
void stageNetwork(const ConfigIP& ip);
bool take(Update& update);
// Controle entrega o pedido de persistência somente após cortar o motor.
void requestPersistence(const Update& update);
bool takePersistence(Update& update);
bool maintenanceActive();
void finishPersistence();
}
