#pragma once

/**
 * @file cloud_callback.h
 * @author Pascal Buschermöhle
 */

#include <msg/msgs_stamped.h>
#include <util/point_hw.h>
#include <hw/buffer/buffer.h>
#include <eigen3/Eigen/Dense>
#include <util/types.h>


namespace fastsense::preprocessing
{

class Preprocessing
{
public:
    void preprocess_scan(const fastsense::msg::PointCloudStamped& cloud, fastsense::buffer::InputBuffer<PointHW>& scan_points, const Matrix4f& pose);

    void reduction_filter(fastsense::msg::PointCloudStamped& cloud);

    void median_filter(fastsense::msg::PointCloudStamped& cloud, uint8_t window_size);

private:
    uint8_t median_from_array(std::vector<fastsense::msg::Point*> medians);
};

}