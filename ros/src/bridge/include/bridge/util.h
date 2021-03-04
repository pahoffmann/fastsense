#pragma once

#include <ros/ros.h>
#include <util/time.h>

namespace fastsense::bridge
{

inline ros::Time timestamp_to_rostime(const fastsense::util::HighResTimePoint& ts)
{
	auto t_since_epoch = ts.time_since_epoch();
	int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(t_since_epoch).count();
	int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(t_since_epoch).count() % 1'000'000'000ll;
	return ros::Time(sec, nsec);
}

}