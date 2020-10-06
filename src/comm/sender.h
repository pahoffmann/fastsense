#pragma once

/**
 * @author Julian Gaal
 */

#include <string>
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <msg/zmq_converter.h>

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

    template <typename TT = T, std::enable_if_t<! std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    void send(T* data, zmq::send_flags flag = zmq::send_flags::dontwait)
    {
        auto length = sizeof(T);
        zmq::message_t msg(length);
        memcpy(msg.data(), data, length);
        socket_.send(msg, flag);
    }

    template <typename TT = T, std::enable_if_t<std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    void send(const T& data, zmq::send_flags flag = zmq::send_flags::dontwait)
    {
        zmq::multipart_t multi = data.to_zmq_msg();
        multi.send(socket_);
    }

private:
    std::string addr_;
    size_t port_;
    zmq::context_t context_;
    zmq::socket_t socket_;
};

} // namespace fastsense::comm
