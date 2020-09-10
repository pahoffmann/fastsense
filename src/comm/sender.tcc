
using namespace fastsense::comm;

template <typename T>
Sender<T>::Sender(std::string addr, size_t threads) 
    : context_(zmq::context_t(threads)), socket_({}), addr_(addr)
{
    socket_ = zmq::socket_t(context_, zmq::socket_type::push);
    socket_.connect("tcp://" + addr_);
}   

template <typename T>
void Sender<T>::send(T* data) {
    socket_.send(data, sizeof(T));
}