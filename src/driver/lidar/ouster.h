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

namespace fastsense::driver
{

class OusterDriver : public fastsense::util::ProcessThread
{
public:
    using UPtr = std::unique_ptr<OusterDriver>;

    OusterDriver(const std::string& hostname, const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& buffer, ouster::sensor::lidar_mode mode, const std::string& metadata = "", uint16_t lidar_port = 0);

    ~OusterDriver() = default;

    OusterDriver& operator=(const OusterDriver&) = delete;

    OusterDriver& operator=(OusterDriver&&) noexcept = delete;

    OusterDriver(const OusterDriver&) = delete;

    OusterDriver(OusterDriver&&) = delete;

    void start() override;

    fastsense::msg::PointCloudPtrStampedBuffer getScan();

    void fill_scan_full(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points);

    void fill_scan_partial(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points);

protected:

    void thread_run() override;

    std::shared_ptr<ouster::sensor::client> handle_;

    std::string metadata_;

    ouster::sensor::sensor_info info_;

    size_t w_;
    size_t h_;

    ouster::sensor::ColumnWindow column_window_;

    int column_window_length_;

    std::unique_ptr<ouster::sensor::packet_format> pf_ptr_;
    std::unique_ptr<ouster::ScanBatcher> batch_to_scan_ptr_;

    /// function to process laserscan
    std::function<void(ouster::LidarScan& scan, const ouster::LidarScan::Points& ouster_points)> fill_scan_function_;

    /// Buffer to write scans to
    fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudPtrStamped>::Ptr scan_buffer_;

    /// Current scan
    fastsense::msg::PointCloud::Ptr current_scan_;

    /// Target Height
    int target_height_;

    /// Which ring to skip
    int skip_index_;
};

} // namespace fastsense::driver

#endif 