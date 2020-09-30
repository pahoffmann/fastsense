/**
 * @file sender.tcc
 * @author Julian Gaal
 * @date 2020-09-06
 */

#pragma once

namespace fastsense::comm
{

template <typename T>
Sender<T>::Sender(std::string addr, size_t threads) 
    :  addr_{addr}, context_{static_cast<int>(threads)}, socket_{context_, zmq::socket_type::push}
{
    socket_.connect("tcp://" + addr_);
}   

template <typename T>
void Sender<T>::send(T* data, zmq::send_flags flag) {
    socket_.send(data, sizeof(T));
}

} // namespace fastsense::comm
