#include "../../src/driver/imu/imu.h"
#include "../../src/driver/imu/msg/imu_msg.h"
#include "../../src/util/concurrent_ring_buffer.h"

namespace fs = fastsense;
using Buffer = fs::util::ConcurrentRingBuffer<fs::driver::msg::ImuMsg>;
using namespace fs::driver;

int main() {
    Buffer imu_buffer(1000);
    Imu imu(imu_buffer);
    while (true) {
        msg::ImuMsg msg;
        imu_buffer.pop(&msg);
	std::cout << msg;
    };
    return 0;
}
