#include "ouster.h"
#include <util/config/config_manager.h>
#include <util/logging/logger.h>

using namespace fastsense::msg;
using namespace ouster;

namespace fastsense::driver
{

const size_t UDP_BUF_SIZE = 65536;

OusterDriver::OusterDriver(const std::string& hostname,
                           const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& pcl_buffer,
                           const fastsense::msg::ImuStampedBuffer::Ptr& imu_buffer,
                           size_t imu_filter_size,
                           ouster::sensor::lidar_mode mode,
                           const std::string& metadata,
                           uint16_t lidar_port)
: OusterDriver(hostname, pcl_buffer, mode, metadata, lidar_port)
{
    imu_buffer_ = fastsense::msg::ImuStampedBuffer::Ptr(imu_buffer);
    imu_filter_ = std::make_optional<util::SlidingWindowFilter<msg::Imu>>(imu_filter_size);
}

OusterDriver::OusterDriver(const std::string& hostname,
                           const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& pcl_buffer,
                           ouster::sensor::lidar_mode mode,
                           const std::string& metadata,
                           uint16_t lidar_port)
    : handle_{sensor::init_client(hostname, "", mode)}
    , scan_buffer_(pcl_buffer)
    , current_scan_{std::make_shared<PointCloud>()}
    , target_height_{static_cast<size_t>(util::config::ConfigManager::config().lidar.height())}
    , skip_index_{1}
{
    if (!handle_)
    {
        util::logging::Logger::fatal("Cannot initialize ouster client with hostname: ", hostname);
        return;
    }

    metadata_ = sensor::get_metadata(*handle_);
    info_ = sensor::parse_metadata(metadata_);

    w_ = info_.format.columns_per_frame;
    h_ = info_.format.pixels_per_column;

    skip_index_ = h_ / target_height_;

    column_window_ = info_.format.column_window;

    column_window_length_ = (column_window_.second - column_window_.first + w_) % w_ + 1;

    util::logging::Logger::info(
             "  Firmware version:  ", info_.fw_rev
            ,"\n  Serial number:     ", info_.sn
            ,"\n  Product line:      ", info_.prod_line
            ,"\n  Scan dimensions:   ", w_, " x ", h_
            ,"\n  Column window:     [", column_window_.first, ", "
            , column_window_.second, "]"
            ,"\n  Scan lines: ", target_height_, "/", 128);

    pf_ptr_ = std::make_unique<ouster::sensor::packet_format>(sensor::get_format(info_));
    batch_to_scan_ptr_.reset(new ScanBatcher(info_.format.columns_per_frame, *pf_ptr_));

    // if skip_index_ != 1 the target height and height of sensor are different -> fill_scan_partially with every skip_index'th line
    if (skip_index_ != 1)
    {
        fill_scan_function_ = std::bind(&OusterDriver::fill_scan_partial, this, std::placeholders::_1, std::placeholders::_2);
    }
    else
    {
        fill_scan_function_ = std::bind(&OusterDriver::fill_scan_full, this, std::placeholders::_1, std::placeholders::_2);
    }
}

void OusterDriver::start()
{
    if (!handle_)
    {
        util::logging::Logger::error("Ouster client not initisalized! Cannot start ouster driver!");
        return;
    }

    if (!running)
    {
        running = true;
        current_scan_ = std::make_shared<PointCloud>();
        scan_buffer_->clear();
        worker = std::thread(&OusterDriver::thread_run, this);
    }
}

void OusterDriver::thread_run()
{
    // buffer to store raw packet data
    std::unique_ptr<uint8_t[]> lidar_packet_buf(new uint8_t[UDP_BUF_SIZE]);
    LidarScan scan(w_, h_, info_.format.udp_profile_lidar);
    XYZLut lut = ouster::make_xyz_lut(info_);

    while (running)
    {
        sensor::client_state st = sensor::poll_client(*handle_);

        if (st == sensor::EXIT)
        {
            util::logging::Logger::info("Ouster caught signal, exiting");
            running = false;
        }

        if (st & sensor::CLIENT_ERROR)
        {
            util::logging::Logger::info("Sensor client returned error state!");
            util::logging::Logger::info("Ending ouster thread");

            running = false;
            continue;
        }

        if (st & sensor::LIDAR_DATA) 
        {
            if (!sensor::read_lidar_packet(*handle_, lidar_packet_buf.get(), *pf_ptr_))
            {
                util::logging::Logger::error("Failed to read a packet of the expected size!");
                continue;
            }
            
            // batcher will return "true" when the current scan is complete
            if ((*batch_to_scan_ptr_)(lidar_packet_buf.get(), scan)) 
            {
                auto ouster_points = cartesian(scan, lut);

                fill_scan_function_(scan, ouster_points);

                scan_buffer_->push_nb(Stamped<PointCloud::Ptr>{current_scan_, util::HighResTime::now()}, true);
                current_scan_ = std::make_shared<PointCloud>();
            }

            if (st & sensor::IMU_DATA) 
            {
                if (!imu_filter_)
                {
                    continue;
                }

                if (!sensor::read_imu_packet(*handle_, lidar_packet_buf.get(), *pf_ptr_))
                {
                    util::logging::Logger::error("Failed to read a packet of the expected size!");
                    continue;
                }

                constexpr double STANDARD_G = 9.80665;

                double acceleration[3];
                double angular_rate[3]; 
                double magnetic_field[3];

                acceleration[0] = (*pf_ptr_).imu_la_x(lidar_packet_buf.get()) * STANDARD_G;
                acceleration[1] = (*pf_ptr_).imu_la_y(lidar_packet_buf.get()) * STANDARD_G;
                acceleration[2] = (*pf_ptr_).imu_la_z(lidar_packet_buf.get()) * STANDARD_G;

                angular_rate[0] = (*pf_ptr_).imu_av_x(lidar_packet_buf.get()) * M_PI / 180.0;
                angular_rate[1] = (*pf_ptr_).imu_av_y(lidar_packet_buf.get()) * M_PI / 180.0;
                angular_rate[2] = (*pf_ptr_).imu_av_z(lidar_packet_buf.get()) * M_PI / 180.0;

                magnetic_field[0] = 0.0;
                magnetic_field[1] = 0.0;
                magnetic_field[2] = 0.0;

                const auto filtered_msg = (*imu_filter_).update(msg::Imu{acceleration, angular_rate, magnetic_field});
                imu_buffer_->push_nb(msg::ImuStamped{std::move(filtered_msg), util::HighResTime::now()}, true);
            }
        }
    }
}

void OusterDriver::fill_scan_full(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points)
{
    current_scan_->height_ = h_;
    current_scan_->width_ = w_;
    current_scan_->scaling_ = 1.0f;
    current_scan_->points_.resize(scan.w * scan.h);

    for (auto u = 0; u < scan.h; u++)
    {
        for (auto v = 0; v < scan.w; v++)
        {
            const auto xyz = ouster_points.row(u * scan.w + v);

            auto &new_point = current_scan_->points_[u * scan.w + v];
            new_point.x() = xyz(0) * 1000;
            new_point.y() = xyz(1) * 1000;
            new_point.z() = xyz(2) * 1000;
        }
    }
}

void OusterDriver::fill_scan_partial(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points)
{
    size_t target_height_index = 0;

    current_scan_->height_ = target_height_;
    current_scan_->width_ = w_;
    current_scan_->scaling_ = 1.0f;
    current_scan_->points_.resize(scan.w * target_height_);

    for (auto u = 0; u < scan.h; u++)
    {
        if (u % skip_index_ == 0)
        {
            for (auto v = 0; v < scan.w; v++)
            {
                const auto xyz = ouster_points.row(u * scan.w + v);

                auto &new_point = current_scan_->points_[target_height_index * scan.w + v];
                new_point.x() = xyz(0) * 1000;
                new_point.y() = xyz(1) * 1000;
                new_point.z() = xyz(2) * 1000;
            }
            target_height_index++;
        }
    }
}

} // fastsense::driver