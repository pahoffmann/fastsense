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

TransformBridge::TransformBridge(const std::string& board_addr)
    :   BridgeBase{board_addr},
        ProcessThread{},
        broadcaster{},
        broadcaster_thread{},
        mtx{},
        transform_data{}
{
    transform_data.transform.rotation.w = 1.0;
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
            std::cerr << "VELO " << e.what() << '\n';
        }
    }
}

void TransformBridge::convert()
{
    std::lock_guard guard(mtx);
    transform_data.transform.rotation.x = msg_.data_.rotation.x();
    transform_data.transform.rotation.y = msg_.data_.rotation.y();
    transform_data.transform.rotation.z = msg_.data_.rotation.z();
    transform_data.transform.rotation.w = msg_.data_.rotation.w();
    transform_data.transform.translation.x = msg_.data_.translation.x() * 0.001;
    transform_data.transform.translation.y = msg_.data_.translation.y() * 0.001;
    transform_data.transform.translation.z = msg_.data_.translation.z() * 0.001;
}

void TransformBridge::publish()
{
    std::lock_guard guard(mtx);
    transform_data.header.stamp = timestamp_to_rostime(msg_.timestamp_);
    transform_data.header.frame_id = "map";
    transform_data.child_frame_id = "base_link";

    broadcaster.sendTransform(transform_data);    
    ROS_INFO_STREAM("Transform published\n");
}

void TransformBridge::broadcast()
{
    while (running && ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        publish();
    }
}