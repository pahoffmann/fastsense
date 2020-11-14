#pragma once

/**
 * @file queue_bridge.h
 * @author Marcel Flottmann
 * @date 2020-10-06
 */

#include <util/time.h>
#include <util/concurrent_ring_buffer.h>
#include <util/process_thread.h>
#include <comm/sender.h>

namespace fastsense::comm
{

template<typename T_QUEUE, typename T_MSG>
class QueueBridgeBase : public util::ProcessThread
{
public:
    using BufferType = std::shared_ptr<util::ConcurrentRingBuffer<T_QUEUE>>;

protected:
    BufferType in_;
    BufferType out_;

    Sender<T_MSG> sender_;

    virtual void duplicate() = 0;

public:
    QueueBridgeBase(const BufferType& in, const BufferType& out, uint16_t port) :
        in_{in}, out_{out}, sender_{port}
    {}

    void start() override
    {
        if (!running)
        {
            running = true;
            worker = std::thread(&QueueBridgeBase::duplicate, this);
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

    ~QueueBridgeBase() = default;
};

template<typename T, bool FORCE>
class QueueBridge : public QueueBridgeBase<T, T>
{
public:
    using QueueBridgeBase<T, T>::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void duplicate() override
    {
        while (this->running)
        {
            T val;
            this->in_->pop(&val);
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
            this->sender_.send(val);
        }
    }
};

template<typename T, bool FORCE>
class QueueBridge<std::shared_ptr<T>, FORCE> : public QueueBridgeBase<std::shared_ptr<T>, T>
{
public:
    using QueueBridgeBase<std::shared_ptr<T>, T>::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void duplicate() override
    {
        while (this->running)
        {
            std::shared_ptr<T> val;
            this->in_->pop(&val);
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
            this->sender_.send(*val);
        }
    }
};

template<typename T, bool FORCE>
class QueueBridge<std::pair<T, util::HighResTimePoint>, FORCE> : public QueueBridgeBase<std::pair<T, util::HighResTimePoint>, T>
{
public:
    using QueueBridgeBase<std::pair<T, util::HighResTimePoint>, T>::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void duplicate() override
    {
        while (this->running)
        {
            std::pair<T, util::HighResTimePoint> val;
            this->in_->pop(&val);
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
            this->sender_.send(val.first);
        }
    }
};

template<typename T, bool FORCE>
class QueueBridge<std::pair<std::shared_ptr<T>, util::HighResTimePoint>, FORCE> : public QueueBridgeBase<std::pair<std::shared_ptr<T>, util::HighResTimePoint>, T>
{
public:
    using QueueBridgeBase<std::pair<std::shared_ptr<T>, util::HighResTimePoint>, T>::QueueBridgeBase;
    ~QueueBridge() = default;

protected:
    void duplicate() override
    {
        while (this->running)
        {
            std::pair<std::shared_ptr<T>, util::HighResTimePoint> val;
            this->in_->pop(&val);
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
            this->sender_.send(*val.first);
        }
    }
};

} // namespace fastsense::comm