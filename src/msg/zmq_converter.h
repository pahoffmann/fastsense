#pragma once

// TODO authors

#include <zmq.hpp>
#include <zmq_addon.hpp>

namespace fastsense::msg
{

struct ZMQConverter
{
    ZMQConverter() = default;
    virtual ~ZMQConverter() = default;
    virtual void from_zmq_msg(zmq::multipart_t& msg) = 0;
    virtual zmq::multipart_t to_zmq_msg() const = 0;
};

} // namespace fastsense::msg
