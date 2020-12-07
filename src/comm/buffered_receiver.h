#pragma once

/**
 * @file buffered_receiver.h
 * @author Julian Gaal, Pascal Buschermoehle
 */

#include <util/time.h>
#include <util/process_thread.h>
#include <util/concurrent_ring_buffer.h>
#include <comm/receiver.h>
#include <msg/imu.h>
#include <msg/point_cloud_stamped.h>
#include <iostream>

namespace fastsense::comm
{

/**
 * @brief BufferedReceiver: Receives messages via Receiver and writes data into buffer
 * 
 * @tparam BUFF_T Buffer Type
 * @tparam RECV_T Receive Type
 */
template <typename BUFF_T, typename RECV_T>
class BufferedReceiver : public util::ProcessThread
{
public:
    /**
     * @brief Delete copy constructor
     */
    BufferedReceiver(const BufferedReceiver&) = delete;


    /**
     * @brief Delete move constructor
     */
    BufferedReceiver(BufferedReceiver&&) = delete;


    /**
     * @brief Delete assignment operator
     */
    BufferedReceiver& operator=(BufferedReceiver const&) = delete;

    /**
     * @brief Destroy the Buffered Receiver object
     */
    virtual ~BufferedReceiver() = default;

    /**
     * @brief 'receive' receives one message
     * and is called from an endless loop in thread_run
     */
    virtual void receive() = 0;

    /**
     * @brief thread_run receives until the end of time
     * 
     */
    [[noreturn]]
    void thread_run() override
    {
        for (;;)
        {
            receive();
        }
    }

protected:
    /**
     * @brief Construct a new Buffered Receiver object
     * 
     * @param addr address the receiver connects to
     * @param port port the receiver connects to 
     * @param buffer buffer to write the incoming messages
     */
    BufferedReceiver(const std::string& addr, uint16_t port, std::shared_ptr<util::ConcurrentRingBuffer<BUFF_T>> buffer)
    : receiver_{addr, port}
    , buffer_{buffer}
    {}

    /// Receiver that's used to get data
    Receiver<RECV_T> receiver_;

    /// Received data is written into buffer
    std::shared_ptr<util::ConcurrentRingBuffer<BUFF_T>> buffer_;
    
    /// individual message that is received
    RECV_T msg_;
};

/**
 * @brief BufferedImuStampedReceiver: receives imu message, write imu message into buffer
 */
class BufferedImuStampedReceiver : public BufferedReceiver<msg::ImuStamped, msg::ImuStamped>
{
public:
    /**
     * @brief Construct a new Buffered ImuStamped Receiver object
     * 
     * @param addr address the receiver connects to
     * @param port port the receiver connects to 
     * @param buffer buffer to write the incoming messages
     */
    BufferedImuStampedReceiver(const std::string& addr, uint16_t port, msg::ImuStampedBuffer::Ptr buffer)
    : BufferedReceiver{addr, port, buffer}
    {}

    /**
     * @brief Destroy the Buffered Imu Stamped Receiver object
     */
    virtual ~BufferedImuStampedReceiver() = default;

    /**
     * @brief Receive ImuStamped and write into buffer
     */
    void receive() override
    {
        receiver_.receive(msg_);
        buffer_->push_nb(std::move(msg_));
    }

    using UPtr = std::unique_ptr<BufferedImuStampedReceiver>;
};

/**
 * @brief BufferedPclStampedReceiver: Receiver PointCloudStamped, write to PointCloud*Ptr*Stamped Buffer
 * 
 */
class BufferedPclStampedReceiver : public BufferedReceiver<msg::PointCloudPtrStamped, msg::PointCloudStamped>
{
public:
    /**
     * @brief Construct a new Buffered Pcl Stamped Receiver object
     * 
     * @param addr address the receiver connects to
     * @param port port the receiver connects to 
     * @param buffer buffer to write the incoming messages
     */
    BufferedPclStampedReceiver(const std::string& addr, uint16_t port, msg::PointCloudPtrStampedBuffer::Ptr buffer)
    : BufferedReceiver{addr, port, buffer}
    {}

    /**
     * @brief Destroy the Buffered Pcl Stamped Receiver object
     */
    ~BufferedPclStampedReceiver() override = default;

    /**
     * @brief Receive PointCloudStamped, convert to PointCloud*Ptr*Stamped
     */
    void receive() override
    {
        receiver_.receive(msg_);
        auto& [ pcl, ts ] = msg_;
        buffer_->push_nb(msg::PointCloudPtrStamped{std::make_shared<msg::PointCloud>(std::move(pcl)), ts });
    }

    using UPtr = std::unique_ptr<BufferedPclStampedReceiver>;
};


} // namespace fastsense::comm