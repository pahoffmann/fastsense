#pragma once
/**
 * @author Julian Gaal
 */

namespace fastsense::comm
{

template <typename T>
Receiver<T>::Receiver(int port, size_t threads)
    : context_(threads), socket_(context_, zmq::socket_type::pull), port_(port)
{
    std::string address = "tcp://*:" + std::to_string(port_);
    socket_.bind(address);
}

template <typename T>
T Receiver<T>::receive(zmq::recv_flags flag)
{
    zmq::message_t msg;
    socket_.recv(msg, flag);

    T target;
    memcpy(&target, msg.data(), sizeof(T));
    return target;
}

} // namespace fastsense::comm