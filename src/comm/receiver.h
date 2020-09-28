#pragma once

#include <zmq.hpp>
#include <msg/point_cloud.h>

namespace fastsense::comm
{

template <typename T>
class Receiver
{
public:
    Receiver(int port, size_t threads = 1);
    virtual ~Receiver() = default;
    T receive(zmq::recv_flags flags = zmq::recv_flags::none);
    void receive(T& target, zmq::recv_flags flag = zmq::recv_flags::none);
    
    inline int getPort() const
    {
        return port_;
    }
private:
    zmq::context_t context_;
    zmq::socket_t socket_;
    int port_;
};

} // namespace fastsense::comm

#include <comm/receiver.tcc>