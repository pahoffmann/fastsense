#include "imu.h"
#include "msg/imu_msg.h"
#include "../../util/ring_buffer.h"

int main() {
    auto& imu_buffer = ring_buffer<ImuMsg>::create_ring_buffer(100);
    phidgets::Imu imu = phidgets::Imu(imu_buffer);
    while (true) {
        ImuMsg msg;
        imu_buffer.pop(&msg);
        std::cout << msg;
    };
    return 0;
}
