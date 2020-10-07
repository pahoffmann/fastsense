/**
 * @file queue_bridge.h
 * @author Marcel Flottmann
 * @date 2020-10-06
 */

#pragma once

#include <util/concurrent_ring_buffer.h>
#include <util/process_thread.h>
#include <comm/sender.h>

namespace fastsense::comm
{

template<typename T_QUEUE, typename T_MSG>
class QueueBridgeBase
{
public:
    using BufferType = std::shared_ptr<util::ConcurrentRingBuffer<T_QUEUE>>;

protected:
    BufferType in_;
    BufferType out_;

    Sender<T_MSG> sender;

    virtual void duplicate() = 0;

public:
    QueueBridge(const BufferType& in, const BufferType& out, uint16_t port) :
        in_{in}, out_{out}, sender{port}
    {}

    void start() override
    {
        if (!running)
        {
            running = true;
            worker = std::thread(&QueueBridge::duplicate, this);
        }
    }

    void stop() override
    {
        if (running)
        {
            running = false;
            worker.join();
        }
    }

    ~QueueBridge() = default;
};

template<typename T, bool FORCE>
class QueueBridge : public QueueBridgeBase<T, T>
{
public:
    using QueueBridgeBase::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void duplicate() override
    {
        while (running)
        {
            T val;
            in_->pop(&val);
            if constexpr (FORCE)
            {
                out_->push_nb(val, true);
            }
            else
            {
                out_->push(val);
            }
            sender.send(val);
        }
    }
};

template<typename T, bool FORCE>
class QueueBridge<std::shared_ptr<T>, FORCE> : public QueueBridgeBase<std::shared_ptr<T>, T>
{
public:
    using QueueBridgeBase::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void duplicate() override
    {
        while (running)
        {
            std::shared_ptr<T> val;
            in_->pop(&val);
            if constexpr (FORCE)
            {
                out_->push_nb(val, true);
            }
            else
            {
                out_->push(val);
            }
            sender.send(*val);
        }
    }
};

} // namespace fastsense::comm