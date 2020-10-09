#pragma once

/**
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <zmq.hpp>
#include <msg/point_cloud.h>
#include <msg/zmq_converter.h>
#include <comm/zmq_context_manager.h>

namespace fastsense::comm
{

template <typename T>
class Receiver
{
public:
    Receiver(std::string addr, uint16_t port)
        :   socket_(ZMQContextManager::getContext(), zmq::socket_type::sub)
    {
        socket_.connect("tcp://" + addr + ":" + std::to_string(port));
        socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    }

    ~Receiver() = default;

    template <typename TT = T, std::enable_if_t<!std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    T receive(zmq::recv_flags flags = zmq::recv_flags::none)
    {
        T target;
        receive(target, flags);
        return target;
    }

    template <typename TT = T, std::enable_if_t<!std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    void receive(T& target, zmq::recv_flags flags = zmq::recv_flags::none)
    {
        zmq::message_t msg;
        socket_.recv(msg, flags);
        memcpy(&target, msg.data(), sizeof(T));
    }

    template <typename TT = T, std::enable_if_t<std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    T receive()
    {
        T target;
        receive(target);
        return target;
    }

    template <typename TT = T, std::enable_if_t<std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    void receive(T& target)
    {
        zmq::multipart_t multi;
        multi.recv(socket_);
        target.from_zmq_msg(multi);
    }

private:
    zmq::socket_t socket_;
};

} // namespace fastsense::comm
