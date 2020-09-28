/**
 * @file receiver.tcc
 * @author Julian Gaal
 * @date 2020-09-06
 */
#pragma once

namespace fastsense::comm {

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

template <typename T>
bool Receiver<T>::receive(T& target, zmq::recv_flags flag) 
{
    zmq::message_t msg;
    if (auto bytes_recv = socket_.recv(msg, flag))
    {
        memcpy(&target, msg.data(), sizeof(T));
        return true;
    }

    return false;
}

} // namespace fastsense::comm
