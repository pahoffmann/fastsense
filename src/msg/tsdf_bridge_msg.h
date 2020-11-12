#pragma once

/**
 * @file tsdf_bridge_msg.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include "zmq_converter.h"
#include <vector>
#include <algorithm>

#include <util/point.h>

namespace fastsense::msg
{

struct TSDFBridgeMessage : public ZMQConverter
{
    TSDFBridgeMessage() = default;
    ~TSDFBridgeMessage() override = default;

    float tau_;
    Vector3i size_;
    Vector3i pos_;
    Vector3i offset_;
    std::vector<std::pair<int, int>> tsdf_data_;

    void from_zmq_msg(zmq::multipart_t& msg)
    {
        tau_ = msg.poptyp<float>();
        size_ = msg.poptyp<Vector3i>();
        pos_ = msg.poptyp<Vector3i>();
        offset_ = msg.poptyp<Vector3i>();

        zmq::message_t tsdf_data_msg = msg.pop();
        size_t n_tsdf_values = tsdf_data_msg.size() / sizeof(std::pair<int, int>);
        tsdf_data_.clear();
        tsdf_data_.reserve(n_tsdf_values);
        std::copy_n(static_cast<std::pair<int, int>*>(tsdf_data_msg.data()), n_tsdf_values, std::back_inserter(tsdf_data_));
    }

    zmq::multipart_t to_zmq_msg() const
    {
        zmq::multipart_t multi;
        multi.addtyp(tau_);
        multi.addtyp(size_);
        multi.addtyp(pos_);
        multi.addtyp(offset_);
        multi.add(zmq::message_t(tsdf_data_.begin(), tsdf_data_.end()));
        return multi;
    }
};

} // namespace fastsense::msg