#pragma once

/**
 * @author Julian Gaal
 */

#include <string>
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <msg/zmq_converter.h>
#include <comm/zmq_context_manager.h>

namespace fastsense::comm
{

template <typename T>
class Sender
{
public:
    Sender(uint16_t port)
        :   socket_{ZMQContextManager::getContext(), zmq::socket_type::pub}
    {
        socket_.bind("tcp://*:" + std::to_string(port));
        socket_.setsockopt(ZMQ_SNDHWM, 2);
    }

    ~Sender() = default;

    template <typename TT = T, std::enable_if_t<!std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    void send(const T& data, zmq::send_flags flag = zmq::send_flags::dontwait)
    {
        auto length = sizeof(T);
        zmq::message_t msg(length);
        memcpy(msg.data(), &data, length);
        socket_.send(msg, flag);
    }

    template <typename TT = T, std::enable_if_t<std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    void send(const T& data)
    {
        zmq::multipart_t multi = data.to_zmq_msg();
        multi.send(socket_);
    }

private:
    zmq::socket_t socket_;
};

} // namespace fastsense::comm
