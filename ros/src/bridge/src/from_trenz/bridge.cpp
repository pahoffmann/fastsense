/**
 * @file bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-06
 */


#include <chrono>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <bridge/from_trenz/tsdf_bridge.h>
#include <bridge/from_trenz/imu_bridge.h>
#include <bridge/from_trenz/velodyne_bridge.h>
#include <bridge/from_trenz/transform_bridge.h>

namespace fs = fastsense;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "from_trenz_bridge");
    ros::NodeHandle n("~");

    std::string board_addr;
    if(!ros::param::get("~board_addr", board_addr))
    {
        board_addr = "192.168.1.123"; // default hardware board
    }
    ROS_INFO_STREAM("Board address is " << board_addr);

    // Init pose path: publisher, tf listener
    auto pose_hist_pub = n.advertise<visualization_msgs::MarkerArray>("pose_path", 1000);
    visualization_msgs::MarkerArray markers;
    uint32_t shape = visualization_msgs::Marker::CUBE;
    size_t n_marker = 0;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    fs::bridge::TSDFBridge tsdf_bridge{n, board_addr};
    fs::bridge::ImuBridge imu_bridge{n, board_addr};
    fs::bridge::VelodyneBridge velodyne_bridge{n, board_addr};
    fs::bridge::TransformBridge transform_bridge{board_addr};
    
    tsdf_bridge.start();
    imu_bridge.start();
    velodyne_bridge.start();
    transform_bridge.start();

    ros::Rate rate(10.0);
    while(ros::ok())
    {
        geometry_msgs::TransformStamped transform_stamped;
        
        try 
        {
            transform_stamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            const auto& [header, child_frame_id, transform] = transform_stamped;

            visualization_msgs::Marker marker;

            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            marker.ns = "basic_shapes";
            marker.id = n_marker++;

            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
            marker.type = shape;

            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            marker.action = visualization_msgs::Marker::ADD;

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = transform.translation.x;
            marker.pose.position.y = transform.translation.y;
            marker.pose.position.z = transform.translation.z;
            marker.pose.orientation.x = transform.rotation.x;
            marker.pose.orientation.y = transform.rotation.y;
            marker.pose.orientation.z = transform.rotation.z;
            marker.pose.orientation.w = transform.rotation.w;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration();


            markers.markers.push_back(marker);
            pose_hist_pub.publish(markers);

            ROS_INFO_STREAM("Pose published (based on transform, Marker Array)");
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
    }

    tsdf_bridge.stop();
    imu_bridge.stop();
    velodyne_bridge.stop();
    transform_bridge.stop();

    return 0;
}
