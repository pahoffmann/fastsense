#pragma once

#include <ros/ros.h>
#include <util/time.h>

namespace fastsense::bridge
{

ros::Time timestamp_to_rostime(const fastsense::util::HighResTimePoint& ts)
{
    auto t_since_epoch = ts.time_since_epoch();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(t_since_epoch).count();
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(t_since_epoch).count() % 1'000'000;
    return ros::Time(sec, nsec);
}

}