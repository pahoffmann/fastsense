#ifndef OUSTER_H
#define OUSTER_H

#include <util/time.h>
#include <msg/point_cloud.h>
#include <util/process_thread.h>
#include <util/concurrent_ring_buffer.h>

#include "ouster/ouster/build.h"
#include "ouster/ouster/client.h"
#include "ouster/ouster/lidar_scan.h"
#include "ouster/ouster/types.h"

#include <msg/imu.h>
#include <util/filter.h>

#include <optional>

namespace fastsense::driver
{

class OusterDriver : public fastsense::util::ProcessThread
{
public:
    using UPtr = std::unique_ptr<OusterDriver>;

    /**
     * Ouster Driver Lidar only constructor
     * @param hostname hostname of ouster
     * @param pcl_buffer buffer for pointcloud
     * @param mode lidar mode: 512x10, 512x20, 1024x10, 1024x20, or 2048x10
     * @param metadata metadata path
     * @param lidar_port networking port
     */
    OusterDriver(const std::string& hostname,
                 const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& pcl_buffer,
                 ouster::sensor::lidar_mode mode,
                 const std::string& metadata = "",
                 uint16_t lidar_port = 0);

    /**
     * Ouster Driver Lidar+IMU constructor
     * @param hostname hostname of ouster
     * @param pcl_buffer buffer for pointcloud
     * @param imu_buffer buffer for imu
     * @param filter_size filter width of imu
     * @param mode lidar mode: 512x10, 512x20, 1024x10, 1024x20, or 2048x10
     * @param metadata metadata path
     * @param lidar_port networking port
     */
    OusterDriver(const std::string& hostname,
                 const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& pcl_buffer,
                 const fastsense::msg::ImuStampedBuffer::Ptr& imu_buffer,
                 size_t filter_size,
                 ouster::sensor::lidar_mode mode,
                 const std::string& metadata = "",
                 uint16_t lidar_port = 0);

    ~OusterDriver() override = default;

    OusterDriver& operator=(const OusterDriver&) = delete;

    OusterDriver& operator=(OusterDriver&&) noexcept = delete;

    OusterDriver(const OusterDriver&) = delete;

    OusterDriver(OusterDriver&&) = delete;

    /// start sensor
    void start() override;

    fastsense::msg::PointCloudPtrStampedBuffer getScan();

    /**
     * Fill scan will all points
     * @param scan Ouster Scan format
     * @param ouster_points Ouster points helper structure
     */
    void fill_scan_full(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points);

    /**
     * Fill scan will every nth line, where n = max_height/target_height (see config.json)
     * skip the rest to produce scan of dimensions target_height * default_sensor_width
     * @param scan Ouster Scan format
     * @param ouster_points Ouster points helper structure
    */
    void fill_scan_partial(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points);

protected:

    void thread_run() override;

    /// Sensor Handle to communicate with client library
    std::shared_ptr<ouster::sensor::client> handle_;

    /// metadata path
    std::string metadata_;

    /// Sensor info
    ouster::sensor::sensor_info info_;

    /// width of scan (horizontal resolution)
    size_t w_;

    /// height of scan (vertical resolution)
    size_t h_;

    /// Hilfsstruktur fuer hoehe und Breite
    ouster::sensor::ColumnWindow column_window_;

    /// Laenge der comlumn_window
    int column_window_length_;

    /// pointer for raw packets in ouster driver
    std::unique_ptr<ouster::sensor::packet_format> pf_ptr_;

    /// pointer for raw packet in batches to scan converter
    std::unique_ptr<ouster::ScanBatcher> batch_to_scan_ptr_;

    /// function to process laserscan
    std::function<void(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points)> fill_scan_function_;

    /// Buffer to write scans to
    fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudPtrStamped>::Ptr scan_buffer_;

    /// imubuffer that imu data is written into
    fastsense::msg::ImuStampedBuffer::Ptr imu_buffer_;

    /// optional sliding window filter for imu
    std::optional<util::SlidingWindowFilter<msg::Imu>> imu_filter_;

    /// Current scan
    fastsense::msg::PointCloud::Ptr current_scan_;

    /// Target Height
    size_t target_height_;

    /// Which ring to skip
    size_t skip_index_;
};

} // namespace fastsense::driver

#endif 