#include "../../src/driver/phidgets/imu.h"
#include "../../src/driver/phidgets/msg/imu_msg.h"
#include "../../src/util/concurrent_ring_buffer.h"

using namespace fastsense::driver;
using Buffer = ConcurrentRingBuffer<msg::ImuMsg>;

int main() {
    Buffer imu_buffer(1000);
    Imu imu = Imu(imu_buffer);
    while (true) {
        msg::ImuMsg msg;
        imu_buffer.pop(&msg);
        std::cout << msg;
    };
    return 0;
}
