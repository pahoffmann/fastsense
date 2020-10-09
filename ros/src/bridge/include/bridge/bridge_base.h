/**
 * @file bridge.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#pragma once

#include <string>
#include <iostream>
#include <ros/node_handle.h>
#include <comm/receiver.h>
#include <util/process_thread.h>

namespace fastsense::bridge
{   

template <typename T, typename P, int PORT>
class BridgeBase
{
private:
    comm::Receiver<T> receiver_;
    ros::Publisher pub_;

public:
    BridgeBase() = delete;
    inline BridgeBase(ros::NodeHandle& n, const std::string& name, const std::string& board_addr, size_t msg_buffer_size = 1000) 
    :   receiver_{board_addr, PORT}, 
        msg_{},
        pub_{n.advertise<P>(name, msg_buffer_size)} 
    {
    }

    virtual ~BridgeBase() = default;
    
    virtual void run() 
    {
        receiver_.receive(msg_);
        convert();
        publish();
    }

    inline const T& msg() const 
    {
        return msg_;
    }

    inline ros::Publisher& pub()
    {
        return pub_;
    }

protected:
    T msg_;
    virtual void convert() = 0;
    virtual void publish() = 0;
};
    
}