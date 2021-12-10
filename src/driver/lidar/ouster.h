#ifndef OUSTER_H
#define OUSTER_H

#include <util/time.h>
#include <msg/point_cloud.h>
#include <util/process_thread.h>
#include <util/concurrent_ring_buffer.h>

namespace fastsense::driver
{

class OusterDriver : public fastsense::util::ProcessThread
{
public:
    using UPtr = std::unique_ptr<OusterDriver>;

    OusterDriver(const std::string& hostname, const std::string& metadata, const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& buffer, uint16_t lidar_port = 0);

    ~OusterDriver() override;

    OusterDriver& operator=(const OusterDriver&) = delete;

    OusterDriver& operator=(OusterDriver&&) noexcept = delete;

    OusterDriver(const OusterDriver&) = delete;

    OusterDriver(OusterDriver&&) = delete;

    void start() override;

    fastsense::msg::PointCloudPtrStampedBuffer getScan();

protected:

    void thread_run() override;

    void decode_packet();

    uint16_t port_;

    int sockfd_;
};

} // namespace fastsense::driver

#endif 