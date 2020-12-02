#include <mutex>
#include <iomanip>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


#include <util/time.h>
#include <util/point.h>
#include <util/concurrent_ring_buffer.h>

#include <comm/sender.h>

#include <msg/imu.h>
#include <msg/point_cloud_stamped.h>

#include <registration/imu_accumulator.h>

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
        , imu_sender_{4444}
        , pcl_sender_{3333}
    {
        spinner_.start();
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_raw", 1000, &Bridge::imu_callback, this);
        pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, &Bridge::pcl_callback, this);
        ROS_INFO("to_trenz bridge initiated");
    }

    ~Bridge()
    {
        spinner_.stop();
    };

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

        ROS_INFO("Sent imu\n");
    }

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &pcl)
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

        ROS_INFO("Sent pcl\n");
    }

private:
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;
    ros::Subscriber imu_sub_;
    ros::Subscriber pcl_sub_;
    fastsense::registration::ImuAccumulator imu_accumulator_;
    fs::util::ConcurrentRingBuffer<sensor_msgs::ImuConstPtr> imu_receiver_buffer_;
    fs::comm::Sender<fs::msg::ImuStamped> imu_sender_;
    fs::comm::Sender<fs::msg::PointCloudStamped> pcl_sender_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "to_trenz_bridge");
    Bridge b;
    ros::waitForShutdown();
    return 0;
}