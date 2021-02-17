/**
 * @file bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-06
 */

#include <cassert>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>

#include <msg/point_cloud.h>
#include <bridge/from_trenz/tsdf_bridge.h>
#include <bridge/from_trenz/imu_bridge.h>
#include <bridge/from_trenz/velodyne_bridge.h>
#include <bridge/from_trenz/transform_bridge.h>

namespace fs = fastsense;
using namespace std::chrono_literals;

bool imu_correct = false;
bool pcl_correct = false;
bool tf_correct = false;

template <typename T>
bool approx(T val1, T val2, T diff)
{
    return std::abs(val1 - val2) <= diff;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    assert((msg->linear_acceleration.x == 1));
    assert((msg->linear_acceleration.y == 2));
    assert((msg->linear_acceleration.z == 3));
    assert((msg->angular_velocity.x == 4));
    assert((msg->angular_velocity.y == 5));
    assert((msg->angular_velocity.z == 6));
    ROS_INFO_STREAM("Received imu measurement. Correctly converted.");
}

void tf_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    assert((msg->position.x == 1));
    assert((msg->position.y == 1));
    assert((msg->position.z == 1));
    assert((msg->orientation.x == 0));
    assert((msg->orientation.y == 0));
    assert((msg->orientation.z == 0));
    assert((msg->orientation.w == 1));
    ROS_INFO_STREAM("Received pose. Correctly converted.");
}

void pcl_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    fs::msg::PointCloud pcl;
    pcl.points_.push_back({1, 2, 3});
    pcl.points_.push_back({2, 3, 4});
    pcl.points_.push_back({3, 4, 5});
    
    assert((approx(msg->points[0].x, pcl.points_[0][0]/1000.f, 0.001f)));
    assert((approx(msg->points[0].y, pcl.points_[0][1]/1000.f, 0.001f)));
    assert((approx(msg->points[0].z, pcl.points_[0][2]/1000.f, 0.001f)));
    assert((approx(msg->points[1].x, pcl.points_[1][0]/1000.f, 0.001f)));
    assert((approx(msg->points[1].y, pcl.points_[1][1]/1000.f, 0.001f)));
    assert((approx(msg->points[1].z, pcl.points_[1][2]/1000.f, 0.001f)));
    assert((approx(msg->points[2].x, pcl.points_[2][0]/1000.f, 0.001f)));
    assert((approx(msg->points[2].y, pcl.points_[2][1]/1000.f, 0.001f)));
    assert((approx(msg->points[2].z, pcl.points_[2][2]/1000.f, 0.001f)));
    ROS_INFO_STREAM("Received pointcloud. Correctly converted.");
    std::exit(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "from_trenz_bridge");
    ros::NodeHandle n("~");

    std::string board_addr;
    if(!ros::param::get("~board_addr", board_addr))
    {
        board_addr = "127.0.0.1"; // default for tests
    }
    ROS_INFO_STREAM("Board address is " << board_addr);

    int timeout_ros;
    if(!ros::param::get("~timeout", timeout_ros))
    {
        timeout_ros = 100; // default timeout for receiver
    }
    ROS_INFO_STREAM("Timeout is " << timeout_ros);
    auto timeout = std::chrono::milliseconds(static_cast<size_t>(timeout_ros));

    // Init listeners to zeromq messages
    fs::bridge::TSDFBridge tsdf_bridge{n, board_addr, timeout};
    fs::bridge::ImuBridge imu_bridge{n, board_addr, timeout};
    fs::bridge::VelodyneBridge velodyne_bridge{n, board_addr, timeout};
    fs::bridge::TransformBridge transform_bridge{n, board_addr, timeout};
    
    tsdf_bridge.start();
    imu_bridge.start();
    velodyne_bridge.start();
    transform_bridge.start();
    
    ROS_INFO_STREAM("from_trenz bridge started");

    ros::Rate rate(0.1);

    while(ros::ok())
    {   
        ros::spinOnce();
        rate.sleep();
    }

    tsdf_bridge.stop();
    imu_bridge.stop();
    velodyne_bridge.stop();
    transform_bridge.stop();

    ROS_INFO_STREAM("Tests successful");
    ROS_INFO_STREAM("from_trenz bridge stopped");

    return 0;
}
