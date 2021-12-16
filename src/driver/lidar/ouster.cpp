#include "ouster.h"
#include <util/config/config_manager.h>
#include <util/logging/logger.h>

using namespace fastsense::msg;
using namespace ouster;

namespace fastsense::driver
{

const size_t UDP_BUF_SIZE = 65536;

OusterDriver::OusterDriver(const std::string& hostname, const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& buffer, ouster::sensor::lidar_mode mode, const std::string& metadata, uint16_t lidar_port)
: handle_{sensor::init_client(hostname, "", mode)}
, scan_buffer_(buffer)
, current_scan_{std::make_shared<PointCloud>()}
, target_height_{util::config::ConfigManager::config().lidar.rings_reduced()}
, skip_index_{128 / target_height_}
{
    if (!handle_)
    {
        util::logging::Logger::info("Cannot initialize ouster client with hostname: ", hostname);
        return;
    }

    metadata_ = sensor::get_metadata(*handle_);
    info_ = sensor::parse_metadata(metadata_);

    w_ = info_.format.columns_per_frame;
    h_ = info_.format.pixels_per_column;

    column_window_ = info_.format.column_window;

    column_window_length_ = (column_window_.second - column_window_.first + w_) % w_ + 1;

    util::logging::Logger::info(
             "  Firmware version:  ", info_.fw_rev
            ,"\n  Serial number:     ", info_.sn
            ,"\n  Product line:      ", info_.prod_line
            ,"\n  Scan dimensions:   ", w_, " x ", h_
            ,"\n  Column window:     [", column_window_.first, ", "
            , column_window_.second, "]");

    pf_ptr_ = std::make_unique<ouster::sensor::packet_format>(sensor::get_format(info_));
    batch_to_scan_ptr_.reset(new ScanBatcher(info_.format.columns_per_frame, *pf_ptr_));

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
            
                current_scan_->rings_ = 128;
                current_scan_->scaling_ = 1.0f;

                fill_scan_function_(scan, ouster_points);

                scan_buffer_->push_nb(Stamped<PointCloud::Ptr>{current_scan_, util::HighResTime::now()}, true);
                current_scan_ = std::make_shared<PointCloud>();
            }

            if (st & sensor::IMU_DATA) 
            {
            }
        }
    }
}

void OusterDriver::fill_scan_full(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points)
{
    size_t count = 0;
    current_scan_->points_.resize(scan.w * scan.h);

    for (auto u = 0; u < scan.h; u++)
    {
        for (auto v = 0; v < scan.w; v++)
        {
            const auto xyz = ouster_points.row(u * scan.w + v);

            auto &new_point = current_scan_->points_[count];

            new_point.x() = xyz(0) * 1000;
            new_point.y() = xyz(1) * 1000;
            new_point.z() = xyz(2) * 1000;

            ++count;
        }
    }
}

void OusterDriver::fill_scan_partial(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points)
{
    size_t target_height_index = 0;
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