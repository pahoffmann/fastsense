#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

class Bridge
{
public:
    Bridge()
        : nh_{}, spinner_{2}, imu_sub_{}, pcl_sub_{}, imu_mtx_{}, counter_{}
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
        imu_mtx_.lock();
        counter_ += 1;
        imu_mtx_.unlock();

        ROS_INFO("Counter [imu]: %zu", counter_);
    }

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &pcl)
    {
        imu_mtx_.lock();
        counter_ = 0;
        imu_mtx_.unlock();

        ROS_INFO("Counter [pcl]: %zu", counter_);
    }

private:
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;
    ros::Subscriber imu_sub_;
    ros::Subscriber pcl_sub_;
    std::mutex imu_mtx_;
    size_t counter_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "to_trenz_bridge");
    Bridge b;
    ros::waitForShutdown();
    return 0;
}