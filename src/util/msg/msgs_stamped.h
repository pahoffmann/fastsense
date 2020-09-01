#include <chrono>
#include <ostream>
#include <util/time_stamp.h>
#include <util/msg/imu_msg.h>
#include <util/msg/point_cloud.h>

namespace fastsense::util::msg
{

using ImuMsgStamped = std::pair<fastsense::util::msg::ImuMsg, fastsense::util::TimeStamp>;
using PointCloudStamped = std::pair<PointCloud::ptr, fastsense::util::TimeStamp>;

}

//std::ostream& operator<<(std::ostream& os, const fastsense::util::msg::ImuMsgStamped& data) {
//    auto& [msg, time] = data;
//    os << "[ TIME ]\n";
//    os << msg;
//    return os;
//}
