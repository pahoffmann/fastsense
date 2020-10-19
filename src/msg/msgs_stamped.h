#pragma once

/**
 * @file msgs_stamped.h
 * @author Julian Gaal
 */

#include <ostream>

#include <util/time_stamp.h>
#include "imu_msg.h"
#include "point_cloud.h"

namespace fastsense::msg
{

using ImuMsgStamped = std::pair<ImuMsg, fastsense::util::TimeStamp>;
using PointCloudStamped = std::pair<PointCloud::Ptr, fastsense::util::TimeStamp>;

}

//std::ostream& operator<<(std::ostream& os, const fastsense::msg::ImuMsgStamped& data) {
//    auto& [msg, time] = data;
//    os << "[ TIME ]\n";
//    os << msg;
//    return os;
//}
