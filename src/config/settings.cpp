#include "config/settings.h"
#include "state/shared_value.h"
namespace { SharedValue<ConfigSnapshot> activeConfig; }
namespace config {
ConfigSnapshot snapshot() { return activeConfig.snapshot(); }
void publish(const ConfigSnapshot& value) { activeConfig.publish(value); }
}
