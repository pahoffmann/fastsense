#pragma once

#include <ros/ros.h>
#include <util/time.h>

namespace fastsense::bridge
{

inline ros::Time timestamp_to_rostime(const fastsense::util::HighResTimePoint& ts)
{
    std::chrono::nanoseconds t_since_epoch{ts.time_since_epoch()};
    int64_t nsec{t_since_epoch.count()};
    nsec %= 1'000'000'000ll;
    //      s  ms  µs  ns

    uint32_t sec = std::chrono::duration_cast<std::chrono::seconds>(t_since_epoch).count();
    return ros::Time(sec, static_cast<uint32_t>(nsec));
}

}