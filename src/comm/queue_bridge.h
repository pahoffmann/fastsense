#pragma once

/**
 * @file queue_bridge.h
 * @author Marcel Flottmann
 * @date 2020-10-06
 */

#include <util/concurrent_ring_buffer.h>
#include <util/process_thread.h>
#include <comm/sender.h>
#include <util/time_stamp.h>

namespace fastsense::comm
{

template<typename T_QUEUE, typename T_MSG, bool FORCE>
class QueueBridgeBase : public util::ProcessThread
{
public:
    using BufferType = std::shared_ptr<util::ConcurrentRingBuffer<T_QUEUE>>;

protected:
    BufferType in_;
    BufferType out_;

    Sender<T_MSG> sender_;

    void thread_run() override
    {
        T_QUEUE val;
        while (this->running)
        {
            if (!this->in_->pop_nb(&val, DEFAULT_POP_TIMEOUT))
            {
                continue;
            }
            if (this->out_)
            {
                if constexpr (FORCE)
                {
                    this->out_->push_nb(val, true);
                }
                else
                {
                    this->out_->push(val);
                }
            }
            this->send(val);
        }
    }

    virtual void send(const T_QUEUE& val) = 0;

public:
    QueueBridgeBase(const BufferType& in, const BufferType& out, uint16_t port) :
        in_{in}, out_{out}, sender_{port}
    {}

    ~QueueBridgeBase() = default;
};

template<typename T, bool FORCE>
class QueueBridge : public QueueBridgeBase<T, T, FORCE>
{
public:
    using QueueBridgeBase<T, T, FORCE>::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void send(const T& val) override
    {
        this->sender_.send(val);
    }
};

template<typename T, bool FORCE>
class QueueBridge<std::shared_ptr<T>, FORCE> : public QueueBridgeBase<std::shared_ptr<T>, T, FORCE>
{
public:
    using QueueBridgeBase<std::shared_ptr<T>, T, FORCE>::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void send(const std::shared_ptr<T>& val) override
    {
        this->sender_.send(*val);
    }
};

template<typename T, bool FORCE>
class QueueBridge<std::pair<T, util::TimeStamp>, FORCE> : public QueueBridgeBase<std::pair<T, util::TimeStamp>, T, FORCE>
{
public:
    using QueueBridgeBase<std::pair<T, util::TimeStamp>, T, FORCE>::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void send(const std::pair<T, util::TimeStamp>& val) override
    {
        this->sender_.send(val.first);
    }
};

template<typename T, bool FORCE>
class QueueBridge<std::pair<std::shared_ptr<T>, util::TimeStamp>, FORCE> : public QueueBridgeBase<std::pair<std::shared_ptr<T>, util::TimeStamp>, T, FORCE>
{
public:
    using QueueBridgeBase<std::pair<std::shared_ptr<T>, util::TimeStamp>, T, FORCE>::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void send(const std::pair<std::shared_ptr<T>, util::TimeStamp>& val) override
    {
        this->sender_.send(*val.first);
    }
};

} // namespace fastsense::comm