/**
 * @file transform_bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <iterator>
#include <random>
#include <ros/ros.h>
#include <bridge/util.h>
#include <evaluation/SavePoseStamped.h>
#include <bridge/from_trenz/transform_bridge.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace fastsense::bridge;

TransformBridge::TransformBridge(
        ros::NodeHandle& n,
        const std::string& board_addr,
        std::chrono::milliseconds timeout,
        bool discard_timestamp,
        bool save_poses
)
    :   BridgeBase{n, "pose", board_addr, 1000, timeout},
        ProcessThread{},
        broadcaster{},
        broadcaster_thread{},
        mtx{},
        transform_data{},
        first_msg{true},
        pose_path{},
        pose_stamped{},
        discard_timestamp_{discard_timestamp},
        save_poses_{save_poses},
        first_pose_(true)
{
    transform_data.transform.rotation.w = 1.0;

    // set unchanging frames
    pose_path.header.frame_id = "map";
    pose_stamped.header.frame_id = "map";
    transform_data.header.frame_id = "map";
    transform_data.child_frame_id = "base_link";
}

void TransformBridge::start()
{
    if (!running)
    {
        running = true;
        worker = std::thread(&TransformBridge::run, this);
        broadcaster_thread = std::thread(&TransformBridge::broadcast, this);
    }
}

void TransformBridge::stop()
{
    if (running)
    {
        running = false;
        worker.join();
        broadcaster_thread.join();
    }
}

void TransformBridge::run()
{
    while (running && ros::ok())
    {   
        try
        {
            if (receive())
            {
                ROS_DEBUG_STREAM("Received transform\n");
                convert();
                publish();
            }
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("transform bridge error: " << e.what());
        }
    }
}

void TransformBridge::normalize_quaternion(geometry_msgs::Quaternion& q)
{
    float f = 1.0f / std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    q.x = static_cast<double>(msg_.data_.rotation.x() * f);
    q.y = static_cast<double>(msg_.data_.rotation.y() * f);
    q.z = static_cast<double>(msg_.data_.rotation.z() * f);
    q.w = static_cast<double>(msg_.data_.rotation.w() * f);
}


void TransformBridge::convert()
{
    std::lock_guard guard(mtx);

    auto timestamp = timestamp_to_rostime(msg_.timestamp_, discard_timestamp_);

    const auto& scaling = msg_.data_.scaling;

    transform_data.header.stamp = timestamp;
    normalize_quaternion(transform_data.transform.rotation);
    transform_data.transform.translation.x = msg_.data_.translation.x() * 0.001 / scaling;
    transform_data.transform.translation.y = msg_.data_.translation.y() * 0.001 / scaling;
    transform_data.transform.translation.z = msg_.data_.translation.z() * 0.001 / scaling;

    pose_stamped.header.stamp = timestamp;
    pose_stamped.pose.orientation.x = msg_.data_.rotation.x();
    pose_stamped.pose.orientation.y = msg_.data_.rotation.y();
    pose_stamped.pose.orientation.z = msg_.data_.rotation.z();
    pose_stamped.pose.orientation.w = msg_.data_.rotation.w();
    pose_stamped.pose.position.x = msg_.data_.translation.x() * 0.001 / scaling;
    pose_stamped.pose.position.y = msg_.data_.translation.y() * 0.001 / scaling;
    pose_stamped.pose.position.z = msg_.data_.translation.z() * 0.001 / scaling;


    // If identity is received, we are in cloudcallback iteration 1
    // -> reset pose path
    if (msg_.data_.translation.isZero() && msg_.data_.rotation.isApprox(Eigen::Quaternionf::Identity()))
    {
        ROS_WARN("Resetting pose path, registered new iteration");
        pose_path.poses.clear();
    }


    if (save_poses_)
    {
        static auto save_path_pub = n.advertise<evaluation::SavePoseStamped>("/evaluation/save_pose", 1000);
        static evaluation::SavePoseStamped save_pose;
        save_pose.id = "fastsense";

        save_pose.pose_stamped = pose_stamped;

        save_path_pub.publish(save_pose);
    }

    pose_path.header.stamp = timestamp; 
    pose_path.poses.push_back(pose_stamped);

    // RViz crashes if path is longer than 16384 (~13 minutes at 20 Scans/sec)
    // see https://github.com/ros-visualization/rviz/issues/1107
    if (pose_path.poses.size() >= 16380)
    {
        static std::default_random_engine rng((std::random_device())());

        // remove a random pose, but not the start nor a recent pose
        std::uniform_int_distribution dist(1, (int)pose_path.poses.size() - 1000);
        int index = dist(rng);
        pose_path.poses.erase(pose_path.poses.begin() + index);
    }
}

void TransformBridge::publish()
{
    std::lock_guard guard(mtx);

    broadcaster.sendTransform(transform_data);    

    pub().publish(pose_path);
}

void TransformBridge::broadcast()
{
    ros::Rate r(100);

    while (running && ros::ok())
    {
        r.sleep();
        publish();
    }
}