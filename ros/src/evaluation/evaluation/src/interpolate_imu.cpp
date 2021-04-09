#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <random>
#include <queue>
#include <eigen3/Eigen/Dense>



ros::Publisher imu_pub;
std::queue<sensor_msgs::Imu> imu_queue;

void imuCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    imu_queue.push(*imu_msg);
    if(imu_queue.size() != 2)
    {
        return;
    }
    else
    {
        ROS_INFO_STREAM("junge, da sind ja zwei imu sachen alla");

        const sensor_msgs::Imu& first_data = imu_queue.front();
        const sensor_msgs::Imu& last_data = imu_queue.back();
    
        ros::Duration diff = (last_data.header.stamp - first_data.header.stamp);
        auto nsec = diff.toNSec();

        std::cout << "Diff with duration: " << (diff).toSec() << std::endl;
        std::cout << "Diff with duration: " << (diff).toNSec() << std::endl;

        //hardcode, divide into 3 parts
        auto nsec_3 = nsec / 3;

        //calculate new timestamps

        ros::Time stamp_1 = (ros::Time()).fromNSec(last_data.header.stamp.toNSec() + nsec_3);
        ros::Time stamp_2 = (ros::Time()).fromNSec(last_data.header.stamp.toNSec() + 2 * nsec_3);
        

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

        one.header.stamp = stamp_1;
        two.header.stamp = stamp_2;

        one.angular_velocity.x = ang_vel_1.x;
        one.angular_velocity.y = ang_vel_1.y;
        one.angular_velocity.z = ang_vel_1.z;
        two.angular_velocity.x = ang_vel_2.x;
        two.angular_velocity.y = ang_vel_2.y;
        two.angular_velocity.z = ang_vel_2.z;

        imu_pub.publish(first_data);
        imu_pub.publish(one);
        imu_pub.publish(two);
        //imu_pub.publish(last_data); // last data will be first data in the next iteration, as it is kept in the queue

        imu_queue.pop(); // pop the data, after getting the front

        return;
    }

    //interpolate new timestamps based on

    return;
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
    ros::init(argc, argv, "imu_interpolater");
    std::string imu_topic;
    ros::NodeHandle n("~");
    imu_topic = n.param<std::string>("imu", "/imu/data");

    ROS_INFO_STREAM("Got IMU Topic: " << imu_topic);

    ros::Subscriber imu_sub = n.subscribe(imu_topic, 1, imuCallBack);

    imu_pub = n.advertise<sensor_msgs::Imu>("/interpolation_imu", 1);

    ros::spin();

    return 0;
}