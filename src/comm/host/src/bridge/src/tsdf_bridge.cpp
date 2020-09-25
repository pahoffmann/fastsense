#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <zmq.hpp>
#include <msg/pose.h>
#include <comm/receiver.h>

using fastsense::msg::Point;
using fastsense::comm::Receiver;

int main(int argc, char** argv)
{
    Receiver<Point> receiver(5555);

    ros::init(argc, argv, "tsdf_bridge");
    ros::NodeHandle n;

    auto marker_pub = n.advertise<visualization_msgs::Marker>("visu_marker", 1);
    ros::spin();

    return 0;
}
