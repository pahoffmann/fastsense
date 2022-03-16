/**
 * @file bridge.cpp
 * @author Julian Gaal
 */

#include <mutex>
#include <iomanip>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>

#include <util/time.h>
#include <util/point.h>
#include <util/concurrent_ring_buffer.h>

#include <comm/sender.h>

#include <msg/imu.h>
#include <msg/point_cloud.h>
#include <msg/transform.h>

#include <registration/imu_accumulator.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace fs = fastsense;

/**
 * @brief Bridge that sends data to Trenz board
 * 
 * Imu at port 4444
 * PCL at port 3333
 * 
 * For each PCL and IMU, a separate thread is starten and subscribes to /imu/data_raw and
 * /velodyne_points respectively.
 * 
 * Each subscriber converts ROS messages to the corresponding messages on the Trenz Board 
 * and sends them with a nonblocking sender
 */
class Bridge
{
public:
    /**
     * @brief Construct a new Bridge object
     * 
     * Create sender for IMU and PCL
     * 
     * Start Subscribers to Imu and PCL
     */
    Bridge()
        : nh_{}
        , spinner_{3}
        , imu_sub_{}
        , gt_sub_{}
        , pcl1_sub_{}
        , pcl2_sub_{}
        , imu_sender_{4444}
        , gt_sender_{4440}
        , pcl_sender_{3333}
    {
        //spinner_.start();
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_raw", 1000, &Bridge::imu_callback, this);
        pcl1_sub_ = nh_.subscribe<sensor_msgs::PointCloud>("/velodyne_legacy", 1, &Bridge::pcl1_callback, this);
        pcl2_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &Bridge::pcl2_callback, this);
        gt_sub_   = nh_.subscribe<nav_msgs::Odometry>("/base_footprint_pose_ground_truth", 1, &Bridge::gt_callback, this);
        ROS_INFO("to_trenz bridge initiated");
   
        ros::spin();
    }

    /**
     * @brief Destroy the Bridge object
     * 
     * Stop all threads listening for data from ROS 
     */
    ~Bridge()
    {
        //spinner_.stop();
    };

    void gt_callback(const nav_msgs::Odometry::ConstPtr& ground_truth)
    {
        static tf2_ros::TransformBroadcaster broadcaster;
        static tf2_ros::Buffer tf_buffer;
        static tf2_ros::TransformListener tf_listener(tf_buffer);

        geometry_msgs::TransformStamped base_to_scan;

        try
        {
            base_to_scan = tf_buffer.lookupTransform("base_link", "velodyne", ground_truth->header.stamp, ros::Duration(0.5));
        }
        catch (tf2::TransformException& e)
        {
            ROS_WARN_STREAM("Couldn't transform from '" << "velodyne" << "' to '" << "base_link" << "'." );
            ros::Duration(0.1).sleep();
            return;
        }

        tf2::Transform tf_base_to_scan;
        tf2::convert(base_to_scan.transform, tf_base_to_scan);
        
        tf2::Transform tf_map_to_base;
        tf2::convert(ground_truth->pose.pose, tf_map_to_base);

        //std::cout << base_to_scan.transform.translation.x << " " << base_to_scan.transform.translation.z << std::endl;

        auto scanner_transform = tf_base_to_scan * tf_map_to_base;

        geometry_msgs::Transform scanner_pose;
        tf2::convert(scanner_transform, scanner_pose);

        double roll, pitch, yaw;
        tf2::Quaternion tf_quaternion;

        tf2::convert(ground_truth->pose.pose.orientation, tf_quaternion);
        tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
        transform.ang.x() = roll;
        transform.ang.y() = pitch;
        transform.ang.z() = yaw;

        transform.acc.x() = scanner_pose.translation.x; //ground_truth->pose.pose.position.x;
        transform.acc.y() = scanner_pose.translation.y; //ground_truth->pose.pose.position.y;
        transform.acc.z() = scanner_pose.translation.z; //ground_truth->pose.pose.position.z;
    }

    /**
     * @brief ImuCallback waits for Imu Data and converts sensor_msgs/Imu to fastsense/msg/Imu 
     * and sends it via zeromq
     * 
     * @param msg sensor_msgs::Imu
     */
    void imu_callback(const sensor_msgs::ImuConstPtr& msg)
    {
        fs::msg::Imu imu;
        imu.ang.x() = msg->angular_velocity.x;
        imu.ang.y() = msg->angular_velocity.y;
        imu.ang.z() = msg->angular_velocity.z;
        imu.acc.x() = msg->linear_acceleration.x;
        imu.acc.y() = msg->linear_acceleration.y;
        imu.acc.z() = msg->linear_acceleration.z;

        auto tp = fs::util::HighResTimePoint{std::chrono::nanoseconds{msg->header.stamp.toNSec()}};

        imu_sender_.send(fs::msg::ImuStamped{std::move(imu), tp});

        ROS_DEBUG("Sent imu\n");
    }

    /**
     * @brief pcl2_callback waits for sensor_msgs/PointCloud2, converts it to fastsense/msg/PointCloud
     * and sends it via zeromq
     * 
     * @param pcl sensor_msgs::PointCloud2
     */
    void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr &pcl)
    {

        auto tp = fs::util::HighResTimePoint{std::chrono::nanoseconds{pcl->header.stamp.toNSec()}};

        fastsense::msg::PointCloud trenz_pcl;
        auto& trenz_points = trenz_pcl.points_;

        size_t n_points = pcl->width * pcl->height;

        if (pcl->data.empty())
        {
            ROS_WARN_STREAM("Received empty pointcloud");
        }
        else
        {
            trenz_points.resize(n_points);

            auto pcl_start = sensor_msgs::PointCloud2ConstIterator<float>(*pcl, "x");


            #pragma omp parallel for schedule(static)
            for (size_t i = 0; i < n_points; ++i)
            {
                const auto it = pcl_start + i;
                trenz_points[i] = fs::ScanPoint(it[0] * 1000.f, it[1] * 1000.f, it[2] * 1000.f);
            }
        }

        pcl_sender_.send(fs::msg::PointCloudStamped{std::move(trenz_pcl), tp});

        gt_sender_.send(fs::msg::ImuStamped{std::move(transform), tp});

        ROS_DEBUG("Sent pcl2\n");
    }

    /**
     * @brief pcl1_callback waits for sensor_msgs/PointCloud, converts it to fastsense/msg/PointCloud
     * and sends it via zeromq
     * 
     * @param pcl sensor_msgs::PointCloud
     */
    void pcl1_callback(const sensor_msgs::PointCloudConstPtr &pcl)
    {
        auto tp = fs::util::HighResTimePoint{std::chrono::nanoseconds{pcl->header.stamp.toNSec()}};

        fastsense::msg::PointCloud trenz_pcl;
        auto& trenz_points = trenz_pcl.points_;

        size_t n_points = pcl->points.size();

        if (pcl->points.empty())
        {
            ROS_WARN_STREAM("Received empty pointcloud");
        }
        else
        {
            trenz_points.resize(n_points);

            #pragma omp parallel for schedule(static)
            for (size_t i = 0; i < n_points; ++i)
            {
                const auto point = pcl->points[i];
                trenz_points[i] = fs::ScanPoint(point.x * 1000.f, point.y * 1000.f, point.z * 1000.f);
            }
        }

        pcl_sender_.send(fs::msg::PointCloudStamped{std::move(trenz_pcl), tp});

        ROS_DEBUG("Sent pcl1\n");
    }

private:
    /// ROS Nodehandle
    ros::NodeHandle nh_;

    /// Spinner that subscribes on multiple threads
    ros::AsyncSpinner spinner_;

    /// Imu Subscriber
    ros::Subscriber imu_sub_;

    ros::Subscriber gt_sub_;

    /// PointCloud Subscriber
    ros::Subscriber pcl1_sub_;

    /// PointCloud2 Subscriber
    ros::Subscriber pcl2_sub_;

    /// fastsense::msg::ImuStamped sender
    fs::comm::Sender<fs::msg::ImuStamped> imu_sender_;
    fs::comm::Sender<fs::msg::ImuStamped> gt_sender_;

    fs::msg::Imu transform;

    /// fastsense::msg::PointCloud sender
    fs::comm::Sender<fs::msg::PointCloudStamped> pcl_sender_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "to_trenz_bridge");
    Bridge b;

    ROS_INFO("Stared to trenz bridge");

    ros::waitForShutdown();
    return 0;
}