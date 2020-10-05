#pragma once

#include <zmq.hpp>
#include <string>

namespace fastsense::comm
{

template <typename T>
class Sender
{
public:
    Sender(std::string addr,size_t port, size_t threads = 1)
    :   addr_{addr}, 
        port_{port}, 
        context_{static_cast<int>(threads)}, 
        socket_{context_, zmq::socket_type::push}
    {
        socket_.connect("tcp://" + addr_ + ":" + std::to_string(port));
    }

    ~Sender() = default;

    void send(T* data, zmq::send_flags flag = zmq::send_flags::dontwait)
    {
        socket_.send(data, sizeof(T));
    }
private:
    std::string addr_;
    size_t port_;
    zmq::context_t context_;
    zmq::socket_t socket_;
};

} // namespace fastsense::comm
