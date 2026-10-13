#pragma once
#include <IPAddress.h>
namespace ethernet {
bool setup(bool autosteerAvailable);
IPAddress destination();
IPAddress localAddress();
}
