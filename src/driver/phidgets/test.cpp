#include "imu.h"
#include "imu_msg.h"
#include "../../util/ring_buffer.h"

int main() {
    ring_buffer<ImuMsg> buffer = ring_buffer<ImuMsg>(1000);
    phidgets::Imu imu = phidgets::Imu();
    while (true) {};
    return 0;
}
