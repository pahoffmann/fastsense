#pragma once

#include <zmq.hpp>

class Receiver {
    virtual void receive(zmq::message_t &msg, zmq::recv_flags flag) = 0;
};