#pragma once

/**
 * @file zmq_converter.h
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <zmq.hpp>
#include <zmq_addon.hpp>

namespace fastsense::msg
{

struct ZMQConverter
{
    ZMQConverter() = default;
    virtual ~ZMQConverter() = default;
    
    /// delete copy assignment operator
    ZMQConverter& operator=(const ZMQConverter& other) = default;

    /// delete move assignment operator
    ZMQConverter& operator=(ZMQConverter&&) = default;

    /// delete copy constructor
    ZMQConverter(const ZMQConverter&) = default;

    /// delete move constructor
    ZMQConverter(ZMQConverter&&) = default;

    /**
     * @brief Convert into data from incoming zmq multipart message
     * 
     * @param msg zmq multipart
     */
    virtual void from_zmq_msg(zmq::multipart_t& msg) = 0;
    
    /**
     * @brief Convert data into zmq message
     * 
     * @return zmq::multipart_t zmq multipart msg containing data
     */
    virtual zmq::multipart_t to_zmq_msg() const = 0;
};

} // namespace fastsense::msg
