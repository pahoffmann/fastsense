/**
 * @file zmq_context_manager.h
 * @author Marcel Flottmann
 * @date 2020-10-07
 */

#pragma once

#include <zmq.hpp>

namespace fastsense::comm
{

class ZMQContextManager
{
public:
    ZMQContextManager() = delete;
    ~ZMQContextManager() = delete;

    static zmq::context_t& getContext()
    {
        // Create context with one IO thread
        static zmq::context_t context{1};
        return context;
    }
};

} // namespace fastsense::comm
