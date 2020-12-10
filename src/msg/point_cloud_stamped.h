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

struct PointCloudStamped : public Stamped<PointCloud> {
    PointCloudStamped()
    : Stamped{}
    {
    }

    explicit PointCloudStamped(PointCloud pcl, util::HighResTimePoint timestamp = util::HighResTime::now())
    : Stamped{std::move(pcl), timestamp}
    {
    }

    ~PointCloudStamped() override final = default;
};

} // namespace fastsense::msg
