#include <driver/lidar/velodyne.h>
#include <msg/msgs_stamped.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include <sstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_test");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud>("pointcloud", 1000);

    auto buffer = std::make_shared<ConcurrentRingBuffer<fastsense::msg::PointCloudStamped>>(16);
    fastsense::driver::VelodyneDriver v("", 2368, buffer);
    v.start();

    while (ros::ok())
    {
        fastsense::msg::PointCloudStamped scan_stamped;
        buffer->pop(&scan_stamped);

        auto& [scan, time] = scan_stamped;

        sensor_msgs::PointCloud pc;
        pc.header.frame_id = "world";

        std::transform(scan->points.begin(), scan->points.end(), std::back_inserter(pc.points), [] (fastsense::msg::Point in)
        {
            geometry_msgs::Point32 out;
            out.x = in.x * 0.001f;
            out.y = in.y * 0.001f;
            out.z = in.z * 0.001f;
            return out;
        });

        ROS_INFO("Scan: %zu Points", scan->points.size());
        chatter_pub.publish(pc);

        ros::spinOnce();
    }

    v.stop();

    return 0;
}
