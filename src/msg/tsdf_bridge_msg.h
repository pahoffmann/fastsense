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
#include <util/tsdf.h>
#include <msg/stamped.h>

namespace fastsense::msg
{

/**
 * @brief Represents TSDF Bridge Message
 * 
 * Inherits from ZMQConverter, because it contains a dynamic data type (std::vector)
 */
struct TSDFBridgeMessage : public ZMQConverter
{
    /// default constructor
    TSDFBridgeMessage() = default;

    /// default destructor
    ~TSDFBridgeMessage() override = default;

    /// copy assignment operator
    TSDFBridgeMessage& operator=(const TSDFBridgeMessage& other) = default;

    /// default move assignment operator
    TSDFBridgeMessage& operator=(TSDFBridgeMessage&&) noexcept = default;

    /// default copy constructor
    TSDFBridgeMessage(const TSDFBridgeMessage&) = default;

    /// default move constructor
    TSDFBridgeMessage(TSDFBridgeMessage&&) noexcept = default;

    /// truncation distance
    float tau_;

    /// size of map 
    Vector3i size_;

    /// global position
    Vector3i pos_;
    
    /// shift offset
    Vector3i offset_;

    /// actual tsdf data
    std::vector<TSDFEntry> tsdf_data_;

    /**
     * @brief Convert zmq_multipart to TSDFBridgeMessage
     * 
     * @param msg multipart message
     */
    void from_zmq_msg(zmq::multipart_t& msg)
    {
        tau_ = msg.poptyp<float>();
        size_ = msg.poptyp<Vector3i>();
        pos_ = msg.poptyp<Vector3i>();
        offset_ = msg.poptyp<Vector3i>();

        zmq::message_t tsdf_data_msg = msg.pop();
        size_t n_tsdf_values = tsdf_data_msg.size() / sizeof(TSDFEntry);
        tsdf_data_.clear();
        tsdf_data_.reserve(n_tsdf_values);
        std::copy_n(static_cast<TSDFEntry*>(tsdf_data_msg.data()), n_tsdf_values, std::back_inserter(tsdf_data_));
    }

    /**
     * @brief Convert TSDFBridgeMessage to multipart
     * 
     * @return zmq::multipart_t zmw multipart message
     */
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

using TSDFBridgeMessageStamped = Stamped<TSDFBridgeMessage>;

} // namespace fastsense::msg