#include <chrono>
#include <util/msg/imu_msg.h>
#include <util/msg/point_cloud.h>

namespace fastsense::util::msg
{

using ImuMsgStamped = std::pair<fastsense::util::msg::ImuMsg, std::chrono::high_resolution_clock::time_point>;
using PointCloudStamped = std::pair<PointCloud::ptr, std::chrono::high_resolution_clock::time_point>;

}