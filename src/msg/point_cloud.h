#pragma once

/**
 * @author Marcel Flottmann
 */

#include <cstdint>
#include <vector>
#include <memory>

#include <msg/point.h>
#include <msg/zmq_converter.h>

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
    ~PointCloud() = default;
    using Ptr = std::shared_ptr<PointCloud>;

    std::vector<Point> points_;
    uint16_t rings_;

    void from_zmq_msg(zmq::multipart_t& msg)
    {
        rings_ = msg.poptyp<uint16_t>();

        zmq::message_t point_msg = msg.pop();
        size_t n_points = point_msg.size() / sizeof(Point);
        points_.clear();
        points_.reserve(n_points);
        std::copy_n(static_cast<Point*>(point_msg.data()), n_points, std::back_inserter(points_));
    }

    zmq::multipart_t to_zmq_msg() const
    {
        zmq::multipart_t multi;
        multi.addtyp(rings_);
        multi.add(zmq::message_t(points_.begin(), points_.end()));
        return multi;
    }
};

} // namespace fastsense::msg;
