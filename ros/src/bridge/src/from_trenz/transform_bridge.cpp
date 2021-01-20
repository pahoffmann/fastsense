/**
 * @file transform_bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <iterator>
#include <ros/ros.h>
#include <bridge/util.h>
#include <bridge/from_trenz/transform_bridge.h>

using namespace fastsense::bridge;

TransformBridge::TransformBridge(ros::NodeHandle& n, const std::string& board_addr)
    :   BridgeBase{n, "pose", board_addr},
        ProcessThread{},
        broadcaster{},
        broadcaster_thread{},
        mtx{},
        transform_data{},
        first_smg{true},
        pose_path{},
        pose_stamped()
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
            receive();
            ROS_INFO_STREAM("Received transform\n");
            convert();
            publish();
        }
        catch(const std::exception& e)
        {
            std::cerr << "transform bridge error: " << e.what() << '\n';
        }
    }
}

void TransformBridge::convert()
{
    std::lock_guard guard(mtx);

    auto timestamp = ros::Time::now();//timestamp_to_rostime(msg_.timestamp_);

    // set pose_path header (once and only)
    if (first_smg)
    {
        first_smg = false;
        pose_path.header.stamp = timestamp;
    }

    transform_data.header.stamp = timestamp;
    transform_data.transform.rotation.x = msg_.data_.rotation.x();
    transform_data.transform.rotation.y = msg_.data_.rotation.y();
    transform_data.transform.rotation.z = msg_.data_.rotation.z();
    transform_data.transform.rotation.w = msg_.data_.rotation.w();
    transform_data.transform.translation.x = msg_.data_.translation.x() * 0.001;
    transform_data.transform.translation.y = msg_.data_.translation.y() * 0.001;
    transform_data.transform.translation.z = msg_.data_.translation.z() * 0.001;

    pose_stamped.header.stamp = timestamp;
    pose_stamped.pose.orientation.x = msg_.data_.rotation.x();
    pose_stamped.pose.orientation.y = msg_.data_.rotation.y();
    pose_stamped.pose.orientation.z = msg_.data_.rotation.z();
    pose_stamped.pose.orientation.w = msg_.data_.rotation.w();
    pose_stamped.pose.position.x = msg_.data_.translation.x() * 0.001;
    pose_stamped.pose.position.y = msg_.data_.translation.y() * 0.001;
    pose_stamped.pose.position.z = msg_.data_.translation.z() * 0.001;
}

void TransformBridge::publish()
{
    std::lock_guard guard(mtx);

    broadcaster.sendTransform(transform_data);    

    pose_path.poses.push_back(pose_stamped);
    pub().publish(pose_path);
}

void TransformBridge::broadcast()
{
    while (running && ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        publish();
    }
}