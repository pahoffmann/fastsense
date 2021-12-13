#include "ouster.h"

using namespace fastsense::msg;
using namespace ouster;

namespace fastsense::driver
{

const size_t UDP_BUF_SIZE = 65536;

OusterDriver::OusterDriver(const std::string& hostname, const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& buffer, const fastsense::msg::ImuStampedBuffer::Ptr& imu_buffer, size_t imu_filter_size, const std::string& metadata, uint16_t lidar_port) 
: OusterDriver(hostname, buffer, metadata, lidar_port)
{
    imu_buffer_ = fastsense::msg::ImuStampedBuffer::Ptr(imu_buffer);
    imu_filter_ = std::make_optional<util::SlidingWindowFilter<msg::Imu>>(imu_filter_size);
}

OusterDriver::OusterDriver(const std::string& hostname, const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& buffer, const std::string& metadata, uint16_t lidar_port)
: handle_{sensor::init_client(hostname, "")}, scan_buffer_(buffer), current_scan_{std::make_shared<PointCloud>()}
{
    if (!handle_)
    {
        std::cerr << "Cannot initialize ouster client with hostname: " << hostname << std::endl;
        return;
    }

    metadata_ = sensor::get_metadata(*handle_);
    info_ = sensor::parse_metadata(metadata_);

    w_ = info_.format.columns_per_frame;
    h_ = info_.format.pixels_per_column;

    column_window_ = info_.format.column_window;

    column_window_length_ = (column_window_.second - column_window_.first + w_) % w_ + 1;

    std::cout << "  Firmware version:  "   << info_.fw_rev
              << "\n  Serial number:     " << info_.sn
              << "\n  Product line:      " << info_.prod_line
              << "\n  Scan dimensions:   " << w_ << " x " << h_
              << "\n  Column window:     [" << column_window_.first << ", "
              << column_window_.second << "]" << std::endl;

    pf_ptr_ = std::make_unique<ouster::sensor::packet_format>(sensor::get_format(info_));
    batch_to_scan_ptr_.reset(new ScanBatcher(info_.format.columns_per_frame, *pf_ptr_)); 
}

void OusterDriver::start()
{
    if (!handle_)
    {
        std::cerr << "Ouster client not initisalized! Cannot start ouster driver!" << std::endl;
        return;
    }

    if (running == false)
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
            std::cout << "Ouster caught signal, exiting" << std::endl;
            running = false;
        }

        if (st & sensor::CLIENT_ERROR)
        {
            std::cerr << "Sensor client returned error state!" << std::endl;
            std::cerr << "Ending ouster thread" << std::endl;

            running = false;
            continue;
        }

        if (st & sensor::LIDAR_DATA) 
        {
            if (!sensor::read_lidar_packet(*handle_, lidar_packet_buf.get(), *pf_ptr_))
            {
                std::cerr << "Failed to read a packet of the expected size!" << std::endl;
                continue;
            }
            
            // batcher will return "true" when the current scan is complete
            if ((*batch_to_scan_ptr_)(lidar_packet_buf.get(), scan)) 
            {
                auto ouster_points = cartesian(scan, lut);
            
                current_scan_->rings_ = 128;
                current_scan_->scaling_ = 1.0f;

                current_scan_->points_.resize(scan.w * scan.h);

                size_t count = 0;

                for (auto u = 0; u < scan.h; u++) 
                {
                    for (auto v = 0; v < scan.w; v++) 
                    {
                        const auto xyz = ouster_points.row(u * scan.w + v);
                        
                        auto& new_point = current_scan_->points_[count];

                        new_point.x() = xyz(0) * 1000;
                        new_point.y() = xyz(1) * 1000;
                        new_point.z() = xyz(2) * 1000;

                        ++count;
                    }
                }

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
                    std::cerr << "Failed to read a packet of the expected size!" << std::endl;
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

} // fastsense::driver