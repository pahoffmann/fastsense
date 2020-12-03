#pragma once

/**
 * @file receiver.h
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <msg/point_cloud.h>
#include <comm/zmq_context_manager.h>

namespace fastsense::comm
{

/**
 * @brief Receiver wraps zeromq via cppzmq and supports receiving data of type T
 *
 * @tparam T data type that will be received
 */
template <typename T>
class Receiver
{
public:
    /**
     * @brief Construct a new Receiver object
     *
     * @param addr which address receiver should listen to
     * @param port which port receiver should listen to
     */
    Receiver(std::string addr, uint16_t port)
    :   socket_(ZMQContextManager::getContext(), zmq::socket_type::sub)
    {
        if (addr.empty())
        {
            throw std::runtime_error("Can't connect to address ''");
        }
        
        socket_.connect("tcp://" + addr + ":" + std::to_string(port));
        socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    }

    /**
     * @brief Delete copy constructor
     */
    Receiver(const Receiver&) = delete;


    /**
     * @brief Delete move constructor
     */
    Receiver(Receiver&&) = delete;


    /**
     * @brief Delete assignment operator
     */
    Receiver& operator=(Receiver const&) = delete;

    /**
     * @brief Destroy the Receiver object
     */
    virtual ~Receiver() = default;

    /**
     * @brief Receive a message of static size
     *
     * @tparam TT type used to compile only if T does NOT inherit from msg::ZMQConverter (therefore only static members)
     * @param flags ZMQ receive flags, default 'none' -> blocking
     * @return T data type of receiver
     */
    template < typename TT = T, std::enable_if_t < !std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    T receive(zmq::recv_flags flags = zmq::recv_flags::none)
    {
        T target;
        receive(target, flags);
        return target;
    }

    /**
     * @brief Receive a message of static size, by reference
     *
     * @tparam TT type used to compile only if T DOES NOT inherit from msg::ZMQConverter (therefore members of dynamic size)
     * @param target where message is copied to
     * @param flags ZMQ receive flags, default 'none' -> blocking
     */
    template < typename TT = T, std::enable_if_t < !std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    void receive(T& target, zmq::recv_flags flags = zmq::recv_flags::none)
    {
        zmq::message_t msg;
        socket_.recv(msg, flags);
        target = *static_cast<T*>(msg.data());
    }

    /**
     * @brief Receive a message with elements of dynamic size
     *
     * @tparam TT type used to compile ONLY IF T inherits from msg::ZMQConverter (therefore members of dynamic size)
     * @return T data type of receiver
     */
    template <typename TT = T, std::enable_if_t<std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    T receive()
    {
        T target;
        receive(target);
        return target;
    }

    /**
     * @brief Receive a message with elements of dynamic size (by reference)
     *
     * @tparam TT type used to compile ONLY IF T inherits from msg::ZMQConverter (therefore members of dynamic size)
     * @param target where message is copied to
     */
    template <typename TT = T, std::enable_if_t<std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    void receive(T& target)
    {
        zmq::multipart_t multi;
        multi.recv(socket_);
        target.from_zmq_msg(multi);
    }

private:
    /// ZeroMQ socket
    zmq::socket_t socket_;
};

} // namespace fastsense::comm
