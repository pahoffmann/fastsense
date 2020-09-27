#pragma once

namespace fastsense::comm
{

template <typename T>
Sender<T>::Sender(std::string addr, size_t threads) 
    : context_(threads), socket_(context_, zmq::socket_type::push), addr_(addr)
{
    socket_.connect("tcp://" + addr_);
}   

template <typename T>
void Sender<T>::send(T* data) {
    socket_.send(data, sizeof(T));
}

}