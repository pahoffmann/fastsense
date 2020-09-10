#pragma once

#include <zmq.hpp>
#include <string>

namespace fastsense::comm {

template <typename T>
class Sender {
public:
    Sender(std::string addr, size_t threads = 1);
    void send(T* data);
private:
    std::string addr_;
    zmq::context_t context_;
    zmq::socket_t socket_;
};

} // namespace fastsense::comm

#include <comm/sender.tcc>