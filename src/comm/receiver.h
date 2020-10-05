#pragma once

#include <zmq.hpp>
#include <msg/point_cloud.h>

namespace fastsense::comm
{

template <typename T>
class Receiver
{
public:
    Receiver(int port, size_t threads = 1)
    :   context_(threads), 
        socket_(context_, zmq::socket_type::pull), 
        port_(port)
    {
        std::string address = "tcp://*:" + std::to_string(port_);
        socket_.bind(address);
    }

    ~Receiver() = default;

    T receive(zmq::recv_flags flag = zmq::recv_flags::none)
    {
        zmq::message_t msg;
        socket_.recv(msg, flag);

        T target;
        memcpy(&target, msg.data(), sizeof(T));
        return target;
    }
  
    void receive(T& target, zmq::recv_flags flag = zmq::recv_flags::none)
    {
        zmq::message_t msg;
        socket_.recv(msg, flag);
        memcpy(&target, msg.data(), sizeof(T));
    }
    
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
