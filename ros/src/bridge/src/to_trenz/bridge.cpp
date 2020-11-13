#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <msg/point_cloud.h>
#include <registration/imu_accumulator.h>

class Bridge
{
public:
    Bridge()
        : nh_{}, spinner_{2}, imu_sub_{}, pcl_sub_{}, imu_mtx_{}, trenz_pcl_{}
    {
        spinner_.start();
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_raw", 1000, &Bridge::imu_callback, this);
        pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/pointcloud2", 1000, &Bridge::pcl_callback, this);
    }

    ~Bridge()
    {
        spinner_.stop();
    };

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
    {
//        imu_mtx_.lock();
//        imu_mtx_.unlock();

        ROS_INFO("Counter [imu]"); //:", counter_);
    }

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &pcl)
    {
//        imu_mtx_.lock();
//        imu_mtx_.unlock();

        auto& trenz_points = trenz_pcl_.points_;
        trenz_points.clear();
        trenz_points.resize(pcl->data.size());

        for (sensor_msgs::PointCloud2ConstIterator<float> it1(*pcl, "x"); it1 != it1.end(); ++it1)
        {
            trenz_points.emplace_back(it1[0], it1[1], it1[2]);
        }

        ROS_INFO("Counter [pcl]"); //:", counter_);
    }

private:
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;
    ros::Subscriber imu_sub_;
    ros::Subscriber pcl_sub_;
    std::mutex imu_mtx_;
    fastsense::msg::PointCloud trenz_pcl_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "to_trenz_bridge");
    Bridge b;
    ros::waitForShutdown();
    return 0;
}