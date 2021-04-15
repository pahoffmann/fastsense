#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <random>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <thread>

ros::Publisher imu_pub;
std::queue<sensor_msgs::Imu> imu_queue;
std::queue<sensor_msgs::Imu, std::list<sensor_msgs::Imu>> imu_pub_queue;
std::queue<double, std::list<double>> imu_times_queue;

std::mutex imu_mutex;

void imuCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    imu_queue.push(*imu_msg);

    if(imu_queue.size() != 2)
    {
        return;
    }
    else
    {
        const sensor_msgs::Imu& first_data = imu_queue.front();
        const sensor_msgs::Imu& last_data = imu_queue.back();
    
        ros::Duration diff = (last_data.header.stamp - first_data.header.stamp);

        Eigen::Vector3d vec_diff(last_data.angular_velocity.x - first_data.angular_velocity.x,
                                 last_data.angular_velocity.y - first_data.angular_velocity.y,
                                 last_data.angular_velocity.z - first_data.angular_velocity.z);
        vec_diff /= 3.0;

        geometry_msgs::Vector3 ang_vel_1;
        ang_vel_1.x = first_data.angular_velocity.x + vec_diff[0];
        ang_vel_1.y = first_data.angular_velocity.y + vec_diff[1];
        ang_vel_1.z = first_data.angular_velocity.z + vec_diff[2];

        geometry_msgs::Vector3 ang_vel_2;
        ang_vel_2.x = first_data.angular_velocity.x + 2 * vec_diff[0];
        ang_vel_2.y = first_data.angular_velocity.y + 2 * vec_diff[1];
        ang_vel_2.z = first_data.angular_velocity.z + 2 * vec_diff[2];

        sensor_msgs::Imu one = first_data;
        sensor_msgs::Imu two = first_data;

        one.angular_velocity.x = ang_vel_1.x;
        one.angular_velocity.y = ang_vel_1.y;
        one.angular_velocity.z = ang_vel_1.z;
        two.angular_velocity.x = ang_vel_2.x;
        two.angular_velocity.y = ang_vel_2.y;
        two.angular_velocity.z = ang_vel_2.z;
        
        auto part_diff = diff.toSec() / 3.0;

        imu_mutex.lock();
        imu_pub_queue.push(first_data);
        imu_pub_queue.push(one);
        imu_pub_queue.push(two);

        imu_times_queue.push(part_diff);
        imu_times_queue.push(part_diff);
        imu_times_queue.push(part_diff);

        imu_mutex.unlock();
        
        imu_queue.pop(); // pop the data, after getting the front
    }
}

void timedCallback(const ros::TimerEvent&)
{
    if (imu_pub_queue.size() != 0)
    {
        imu_mutex.lock();
        auto to_pub = imu_pub_queue.front();
        auto next_time_out = imu_times_queue.front();
        imu_pub_queue.pop();
        imu_times_queue.pop();
        imu_mutex.unlock();
        
        to_pub.header.stamp = ros::Time::now();
        imu_pub.publish(to_pub);
    }
}

/**
 * This node subscribes to the bagfile of a ... topic and interpolates the imu data 
 *
 * @param argc number of args
 * @param argv args
 * @return 0 if success
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "live_imu_interpolater");
    ros::NodeHandle n("~");
    auto imu_topic = n.param<std::string>("imu", "/imu/data");

    ROS_INFO_STREAM("Got IMU Topic: " << imu_topic);

    ros::Subscriber imu_sub = n.subscribe(imu_topic, 100, imuCallBack);

    imu_pub = n.advertise<sensor_msgs::Imu>("/from_trenz/imu/raw", 100);
    ros::Timer timer = n.createTimer(ros::Duration(1.0 / (73 * 3)), timedCallback);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}