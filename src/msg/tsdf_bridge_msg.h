#pragma once

/**
 * @file bridge_messages.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include "zmq_converter.h"
#include <array>
#include <vector>
#include <algorithm>

namespace fastsense::msg
{

struct TSDFBridgeMessage : public ZMQConverter
{
    float tau_;
    float map_resolution_;
    std::array<int, 3> size_;
    std::array<int, 3> pos_;
    std::array<int, 3> offset_;
    std::vector<std::pair<float, float>> tsdf_data_;

    void from_zmq_msg(zmq::multipart_t& msg)
    {
        tau_ = msg.poptyp<float>();
        map_resolution_ = msg.poptyp<float>();
        size_ = msg.poptyp<std::array<int, 3>>();
        pos_ = msg.poptyp<std::array<int, 3>>();
        offset_ = msg.poptyp<std::array<int, 3>>();

        zmq::message_t tsdf_data_msg = msg.pop();
        size_t n_tsdf_values = tsdf_data_msg.size() / sizeof(std::pair<float, float>);
        tsdf_data_.reserve(n_tsdf_values);
        tsdf_data_.clear();
        std::copy_n(static_cast<std::pair<float,float>*>(tsdf_data_msg.data()), n_tsdf_values, std::back_inserter(tsdf_data_));
    }

    zmq::multipart_t to_zmq_msg() const
    {
        zmq::multipart_t multi;
        multi.addtyp(tau_);
        multi.addtyp(map_resolution_);
        multi.add(zmq::message_t(size_.begin(), size_.end()));
        multi.add(zmq::message_t(pos_.begin(), pos_.end()));
        multi.add(zmq::message_t(offset_.begin(), offset_.end()));
        multi.add(zmq::message_t(tsdf_data_.begin(), tsdf_data_.end()));
        return multi;
    }
};

} // namespace fastsense::msg