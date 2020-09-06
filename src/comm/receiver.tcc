/**
 * @file receiver.tcc
 * @author Julian Gaal
 * @date 2020-09-06
 */

#include <zmq.hpp>
#include <cassert>

using namespace fastsense::comm;

template <typename T>
Receiver<T>::Receiver(int port, size_t threads) 
    : context_(zmq::context_t(threads)), socket_({}), port_(port)
{
    // construct a REP (reply) socket and bind to interface
    socket_ = zmq::socket_t(context_, zmq::socket_type::pull);
    std::string address = "tcp://*:" + std::to_string(port_);
    socket_.bind(address);
}

template <typename T>
T Receiver<T>::receive(zmq::recv_flags flags) 
{
    zmq::message_t msg;
    socket_.recv(msg, zmq::recv_flags::none);

    T target;
    memcpy(&target, msg.data(), sizeof(T));
    return target;
}
