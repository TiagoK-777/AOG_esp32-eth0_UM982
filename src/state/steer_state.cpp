#include "state/steer_state.h"
#include "state/shared_value.h"
namespace { SharedValue<SteerStatus> status; }
namespace state {
SteerStatus steerSnapshot() { return status.snapshot(); }
void publishSteer(const SteerStatus& value) { status.publish(value); }
}
