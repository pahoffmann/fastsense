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
    BufferedReceiver(std::string addr, uint16_t port, std::shared_ptr<util::ConcurrentRingBuffer<BUFF_T>> buffer)
    : receiver_{addr, port}
    , buffer_{buffer}
    {}

    virtual ~BufferedReceiver() = default;

    virtual void receive() = 0;

    void thread_run() override
    {
        for (;;)
        {
            std::cout << "recv attempt\n";
            receive();
            std::cout << "recv done\n";
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
    BufferedImuReceiver(std::string addr, uint16_t port, msg::ImuStampedBufferPtr buffer)
    : BufferedReceiver{addr, port, buffer}
    {}

    virtual ~BufferedImuReceiver() = default;

private:
    void receive() override
    {
        std::cout << "Receive attempt imu\n";
        receiver_.receive(msg_);
        std::cout << "Received imu\n";
        buffer_->push_nb(msg_);
    }

};

class BufferedPCLReceiver : public BufferedReceiver<msg::PointCloudStamped, std::pair<msg::PointCloud, util::HighResTimePoint>>
{
public:
    BufferedPCLReceiver(std::string addr, uint16_t port, msg::PointCloudStampedBufferPtr buffer)
    : BufferedReceiver{addr, port, buffer}
    {}

    ~BufferedPCLReceiver() override = default;

private:
    void receive() override
    {
        std::cout << "Receive attempt pcl\n";
        receiver_.receive(msg_);
        std::cout << "Received pcl\n";
        auto& [ pcl, ts ] = msg_;
        buffer_->push_nb({ std::make_shared<msg::PointCloud>(pcl), ts });
    }
};


} // namespace fastsense::comm