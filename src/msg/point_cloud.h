#pragma once

/**
 * @file point_cloud.h
 * @author Marcel Flottmann
 */

#include <vector>
#include <memory>

#include <util/time.h>
#include <util/point.h>
#include <msg/zmq_converter.h>

#include <util/concurrent_ring_buffer.h>

namespace fastsense::msg
{

/**
 * @brief A point cloud with fixed rings
 *
 * All columns are stored sequentially. A column consists of a point from every ring.
 * So the data is stored as following (C3R4 = column 3 ring 4)
 * [ C1R1 C1R2 C1R3 C1R4 C2R1 C2R2 C2R3 C2R4 ... ]
 */
class PointCloud : public ZMQConverter
{
public:
    PointCloud() : points_{}, rings_{} {}
    ~PointCloud() override = default;
    using Ptr = std::shared_ptr<PointCloud>;

    std::vector<fastsense::ScanPoint> points_;
    uint16_t rings_;

    void from_zmq_msg(zmq::multipart_t& msg) override
    {
        rings_ = msg.poptyp<uint16_t>();

        zmq::message_t point_msg = msg.pop();
        size_t n_points = point_msg.size() / sizeof(fastsense::ScanPoint);
        points_.clear();
        points_.reserve(n_points);
        std::copy_n(static_cast<fastsense::ScanPoint*>(point_msg.data()), n_points, std::back_inserter(points_));
    }

    zmq::multipart_t to_zmq_msg() const override
    {
        zmq::multipart_t multi;
        multi.addtyp(rings_);
        multi.add(zmq::message_t(points_.begin(), points_.end()));
        return multi;
    }
};

using PointCloudStamped = std::pair<PointCloud::Ptr, util::HighResTimePoint>;
using PointCloudStampedBuffer = util::ConcurrentRingBuffer<PointCloudStamped>;
using PointCloudStampedBufferPtr = std::shared_ptr<PointCloudStampedBuffer>;

} // namespace fastsense::msg;
