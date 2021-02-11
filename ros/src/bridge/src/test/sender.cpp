
/**
  * @file from_bridge_local_test.cpp
  * @author julian 
  * @date 1/13/21
 */

#include <thread>
#include <util/time.h>
#include <msg/transform.h>
#include <msg/imu.h>
#include <msg/point_cloud.h>
#include <msg/tsdf_bridge_msg.h>
#include <comm/sender.h>

using namespace fastsense;
using namespace std::chrono_literals;

#define N_TIMES(x) for (int i = 0; i < x; ++i)

int main()
{
    comm::Sender<msg::TransformStamped> transform_sender{8888};
    comm::Sender<msg::ImuStamped> imu_sender{5555};
    comm::Sender<msg::PointCloudStamped> pcl_sender{7777};
    comm::Sender<msg::TSDFBridgeMessage> tsdf_sender{6666};

    for(;;)
    {
        std::this_thread::sleep_for(500ms);
        auto tp = util::HighResTime::now();

        // send transform message
        msg::Transform tf{Quaternionf{1, 0, 0, 0}, Vector3f{1, 1, 1}};
        msg::TransformStamped tf_stamped{tf, tp};
        transform_sender.send(tf_stamped);
        std::cout << "Sent transform message to port 8888\n";

        // send imu message
        msg::LinearAcceleration acc{1, 2, 3};
        msg::AngularVelocity ang{4, 5, 6};
        msg::MagneticField mag{7, 8, 9};
        msg::Imu imu{acc, ang, mag};
        msg::ImuStamped imu_stamped{imu, tp};

        imu_sender.send(imu_stamped);
        std::cout << "Sent imu message to port 5555\n";

        // send Pointcloud
        msg::PointCloud pcl;
        pcl.rings_ = 2;
        pcl.points_.push_back({1, 2, 3});
        pcl.points_.push_back({2, 3, 4});
        pcl.points_.push_back({3, 4, 5});
        msg::PointCloudStamped pcl_stamped{std::move(pcl), tp};
        pcl_sender.send(pcl_stamped);
        std::cout << "Sent pcl message to port 7777\n--\n";
    }

    return 0;
}