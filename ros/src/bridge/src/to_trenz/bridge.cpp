#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <iomanip>
#include <util/time.h>
#include <util/concurrent_ring_buffer.h>
#include <msg/point_cloud.h>
#include <msg/imu.h>
#include <registration/imu_accumulator.h>
#include <msg/registration_input.h>

namespace fs = fastsense;

class Bridge
{
public:
    Bridge()
        : nh_{}
        , spinner_{2}
        , imu_sub_{}
        , pcl_sub_{}
        , imu_receiver_buffer_{100'000}
    {
        spinner_.start();
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_raw", 1000, &Bridge::imu_callback, this);
        pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, &Bridge::pcl_callback, this);
        ROS_INFO("to_trenz subscriber initiated");
    }

    ~Bridge()
    {
        spinner_.stop();
    };

    static bool imu_younger(ros::Time pcl_stamp, ros::Time imu_stamp)
    {
        return (pcl_stamp - imu_stamp).toNSec() >= 0;
    }

    static bool imu_older(ros::Time pcl_stamp, ros::Time imu_stamp)
    {
        return not imu_younger(pcl_stamp, imu_stamp);
    }

    void imu_callback(const sensor_msgs::ImuConstPtr& msg)
    {
        imu_receiver_buffer_.push(msg);
        // ROS_INFO("[imu] received");
    }

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &pcl)
    {
        auto& pcl_stamp = pcl->header.stamp;
        bool first_msg = true;
        ros::Time last_imu_timestamp;

        sensor_msgs::ImuConstPtr imu_msg;

        while (imu_receiver_buffer_.pop_nb(&imu_msg)) {
            if (imu_older(pcl_stamp, imu_msg->header.stamp))
            {
                break;
            }

            if (first_msg)
            {
                last_imu_timestamp = imu_msg->header.stamp;
                first_msg = false;
                continue;
            }

            double acc_time = (imu_msg->header.stamp - last_imu_timestamp).toSec();

            fs::msg::Imu imu;
            imu.ang.x() = imu_msg->angular_velocity.x;
            imu.ang.y() = imu_msg->angular_velocity.y;
            imu.ang.z() = imu_msg->angular_velocity.z;
            imu.acc.x() = imu_msg->linear_acceleration.x;
            imu.acc.y() = imu_msg->linear_acceleration.y;
            imu.acc.z() = imu_msg->linear_acceleration.z;

            imu_accumulator_.update(imu, acc_time);
            last_imu_timestamp = imu_msg->header.stamp;
        }

        fastsense::msg::PointCloud trenz_pcl;
        auto& trenz_points = trenz_pcl.points_;

        if (pcl->data.size() == 0)
        {
            ROS_WARN_STREAM("Received empty pointcloud");
        }
        else
        {
            trenz_points.resize(pcl->data.size());

            for (sensor_msgs::PointCloud2ConstIterator<float> it1(*pcl, "x"); it1 != it1.end(); ++it1) {
                trenz_points.emplace_back(it1[0], it1[1], it1[2]);
            }
        }

        fs::msg::RegistrationInput trenz_msg{imu_accumulator_.combined_transform(), std::move(trenz_pcl)};
        ROS_INFO_STREAM("[pcl] rotation: " << imu_accumulator_);
        imu_accumulator_.reset();
    }

private:
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;
    ros::Subscriber imu_sub_;
    ros::Subscriber pcl_sub_;
    fastsense::registration::ImuAccumulator imu_accumulator_;
    fs::util::ConcurrentRingBuffer<sensor_msgs::ImuConstPtr> imu_receiver_buffer_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "to_trenz_bridge");
    Bridge b;
    ros::waitForShutdown();
    return 0;
}