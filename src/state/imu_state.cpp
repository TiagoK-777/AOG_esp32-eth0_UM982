#include "state/imu_state.h"
#include "state/shared_value.h"
namespace { SharedValue<ImuState> status; }
namespace state {
ImuState imuSnapshot() { return status.snapshot(); }
void publishImu(const ImuState& value) { status.publish(value); }
}
