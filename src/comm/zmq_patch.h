/**
 * @file zmq_patch.h
 * @author Julian Gaal
 * @date 2020-08-09
 * 
 * This file adds things that are only available in newest version of the cppzmq bindings, but are not available in the petalinux version
 */

#pragma once

#include <zmq.h>

namespace zmq {

enum class send_flags : int
{
    none = 0,
    dontwait = ZMQ_DONTWAIT,
    sndmore = ZMQ_SNDMORE
};

enum class recv_flags : int
{
    none = 0,
    dontwait = ZMQ_DONTWAIT
};

template <typename Enumeration>
typename std::underlying_type<Enumeration>::type as_integer(Enumeration const value)
{
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}


} // namespace zmq