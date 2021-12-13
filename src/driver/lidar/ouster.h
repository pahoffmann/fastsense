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

    OusterDriver(const std::string& hostname, const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& buffer, const std::string& metadata = "", uint16_t lidar_port = 0);

    OusterDriver(const std::string& hostname, const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& buffer, const fastsense::msg::ImuStampedBuffer::Ptr& ringbuffer, size_t filter_size, const std::string& metadata = "", uint16_t lidar_port = 0);

    ~OusterDriver() = default;

    OusterDriver& operator=(const OusterDriver&) = delete;

    OusterDriver& operator=(OusterDriver&&) noexcept = delete;

    OusterDriver(const OusterDriver&) = delete;

    OusterDriver(OusterDriver&&) = delete;

    void start() override;

    fastsense::msg::PointCloudPtrStampedBuffer getScan();

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

    /// Buffer to write scans to
    fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudPtrStamped>::Ptr scan_buffer_;

    fastsense::msg::ImuStampedBuffer::Ptr imu_buffer_;

    std::optional<util::SlidingWindowFilter<msg::Imu>> imu_filter_;

    /// Current scan
    fastsense::msg::PointCloud::Ptr current_scan_;
};

} // namespace fastsense::driver

#endif 