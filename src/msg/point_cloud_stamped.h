#pragma once

/**
 * @file point_cloud_stamped.h
 * @author Julian Gaal
 */

#include "stamped.h"
#include "point_cloud.h"
#include "zmq_converter.h"
#include <util/time.h>

namespace fastsense::msg {

struct PointCloudStamped : public Stamped<PointCloud>, public ZMQConverter {
    PointCloudStamped()
    : Stamped{PointCloud{}, util::HighResTime::now()}
    {
    }

    explicit PointCloudStamped(PointCloud pcl, util::HighResTimePoint timestamp = util::HighResTime::now())
    : Stamped{std::move(pcl), timestamp}
    {
    }

    ~PointCloudStamped() override = default;

    void from_zmq_msg(zmq::multipart_t &msg) override
    {
        timestamp_ = msg.poptyp<util::HighResTimePoint>();
        data_.from_zmq_msg(msg);
    }

    [[nodiscard]]
    zmq::multipart_t to_zmq_msg() const override
    {
        zmq::multipart_t multi;
        multi.addtyp(timestamp_);
        multi.append(data_.to_zmq_msg());
        return multi;
    }
};

} // namespace fastsense::msg
