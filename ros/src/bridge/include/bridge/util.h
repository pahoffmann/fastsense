#pragma once

#include <ros/ros.h>
#include <util/time.h>

namespace fastsense::bridge
{

inline ros::Time timestamp_to_rostime(uint64_t ts)
{
	static ros::Time init_time = ros::Time::now();
	
	// ROS calls fromSec in default constructor, so convert to seconds
	ros::Time tstamp(ts / 1000.0);
	
    return ros::Time(init_time.sec + tstamp.sec, init_time.nsec + tstamp.nsec);
}

inline uint64_t rostime_to_timestamp(const ros::Time& stamp)
{
	return static_cast<uint64_t>(stamp.toNSec() / 1000000.0);
}

}