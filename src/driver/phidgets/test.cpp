#include "imu.h"
#include "msg/imu_msg.h"
#include "../../util/concurrent_ring_buffer.h"

using Buffer = ConcurrentRingBuffer<ImuMsg>;

int main() {
    Buffer imu_buffer(1000);
    phidgets::Imu imu = phidgets::Imu(imu_buffer);
    while (true) {
        ImuMsg msg;
        imu_buffer.pop(&msg);
        std::cout << msg;
    };
    return 0;
}
