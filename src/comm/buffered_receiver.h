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
#include <msg/point_cloud.h>
#include <iostream>

namespace fastsense::comm
{

template <typename BUFF_T, typename RECV_T>
class BufferedReceiver : public util::ProcessThread
{
public:
    BufferedReceiver(const std::string& addr, uint16_t port, std::shared_ptr<util::ConcurrentRingBuffer<BUFF_T>> buffer)
    : receiver_{addr, port}
    , buffer_{buffer}
    {}

    virtual ~BufferedReceiver() = default;

    virtual void receive() = 0;

    [[noreturn]]
    void thread_run() override
    {
        for (;;)
        {
            receive();
        }
    }

protected:
    Receiver<RECV_T> receiver_;
    std::shared_ptr<util::ConcurrentRingBuffer<BUFF_T>> buffer_;
    RECV_T msg_;
};

class BufferedImuReceiver : public BufferedReceiver<msg::ImuStamped, msg::ImuStamped>
{
public:
    BufferedImuReceiver(const std::string& addr, uint16_t port, msg::ImuStampedBuffer::Ptr buffer)
    : BufferedReceiver{addr, port, buffer}
    {}

    virtual ~BufferedImuReceiver() = default;

private:
    void receive() override
    {
        receiver_.receive(msg_);
        buffer_->push_nb(std::move(msg_));
    }

};

class BufferedPCLReceiver : public BufferedReceiver<msg::PointCloudPtrStamped, msg::PointCloudStamped>
{
public:
    BufferedPCLReceiver(const std::string& addr, uint16_t port, msg::PointCloudPtrStampedBuffer::Ptr buffer)
    : BufferedReceiver{addr, port, buffer}
    {}

    ~BufferedPCLReceiver() override = default;

private:
    void receive() override
    {
        receiver_.receive(msg_);
        auto& [ pcl, ts ] = msg_;
        buffer_->push_nb(msg::PointCloudPtrStamped{std::make_shared<msg::PointCloud>(std::move(pcl)), ts });
    }
};


} // namespace fastsense::comm