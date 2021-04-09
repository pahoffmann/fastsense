#include <ros/ros.h>
#include <evaluation/SavePoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

ros::Publisher imu_pub;
std::queue<const sensor_msgs::Imu::ConstPtr> imu_queue;

void imuCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    imu_queue.push(imu_msg);
    if(imu_queue.size() != 2)
    {
        return 0;
    }
    else{
        std::cout << "junge, da sind ja zwei imu sachen alla" << std::endl;

        auto& first_data = imu_queue.front();
        imu_queue.pop(); // pop the data, after getting the front

        return 0;
    }

    //interpolate new timestamps based on 

/**
 * This node subscribes to the bagfile of a ... topic and interpolates the imu data 
 *
 * @param argc number of args
 * @param argv args
 * @return 0 if success
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_interpolater");
    std::string imu_topic;
    ros::NodeHandle n("~");
    n.param("imu", imu_topic, "/imu/data_raw");

    std::cout << "Got PCL Topic: " << pcl_topic << std::endl;
    std::cout << "Got IMU Topic: " << imu_topic << std::endl;

    ros::Subscriber imu_sub = n.subscribe(imu_topic, 1, imuCallBack);

    imu_pub = n.advertise<sensor_msgs::Imu>("/interpolation_imu");

    ROS_INFO_STREAM("Node listening to LOAM data at /key_pose_origin");

    ros::spin();

    return 0;
}