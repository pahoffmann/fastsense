#include <chrono>
#include <ostream>
#include <util/time_stamp.h>
#include <msg/imu_msg.h>
#include <msg/point_cloud.h>

namespace fastsense::msg
{

using ImuMsgStamped = std::pair<ImuMsg, fastsense::util::TimeStamp>;
using PointCloudStamped = std::pair<PointCloud::ptr, fastsense::util::TimeStamp>;

}

//std::ostream& operator<<(std::ostream& os, const fastsense::msg::ImuMsgStamped& data) {
//    auto& [msg, time] = data;
//    os << "[ TIME ]\n";
//    os << msg;
//    return os;
//}
