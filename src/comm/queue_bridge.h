#pragma once

/**
 * @file queue_bridge.h
 * @author Marcel Flottmann, Julian Gaal
 * @date 2020-10-06
 */

#include <util/time.h>
#include <util/concurrent_ring_buffer.h>
#include <util/process_thread.h>
#include <comm/sender.h>

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

    virtual ~QueueBridgeBase() = default;
};

template<typename T, bool FORCE>
class QueueBridge : public QueueBridgeBase<T, T, FORCE>
{
public:
    using QueueBridgeBase<T, T, FORCE>::QueueBridgeBase;
    ~QueueBridge() override = default;

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
    ~QueueBridge() override = default;

protected:
    void send(const std::shared_ptr<T>& val) override
    {
        this->sender_.send(*val);
    }
};

template<typename T, bool FORCE>
class QueueBridge<std::pair<T, util::HighResTimePoint>, FORCE> : public QueueBridgeBase<std::pair<T, util::HighResTimePoint>, T, FORCE>
{
public:
    using QueueBridgeBase<std::pair<T, util::HighResTimePoint>, T, FORCE>::QueueBridgeBase;
    ~QueueBridge() override = default;

protected:
    void send(const std::pair<T, util::HighResTimePoint>& val) override
    {
        this->sender_.send(val.first);
    }
};

template<typename T, bool FORCE>
class QueueBridge<std::pair<std::shared_ptr<T>, util::HighResTimePoint>, FORCE> : public QueueBridgeBase<std::pair<std::shared_ptr<T>, util::HighResTimePoint>, T, FORCE>
{
public:
    using QueueBridgeBase<std::pair<std::shared_ptr<T>, util::HighResTimePoint>, T, FORCE>::QueueBridgeBase;
    ~QueueBridge() override = default;

protected:
    void send(const std::pair<std::shared_ptr<T>, util::HighResTimePoint>& val) override
    {
        this->sender_.send(*val.first);
    }
};

/**
 * @brief QueueBridgeBase specialization for PointCloudPtrStamped (trenz to ROS)
 * 
 * @tparam T 
 * @tparam FORCE 
 */
template<bool FORCE>
class QueueBridge<msg::Stamped<std::shared_ptr<msg::PointCloud>>, FORCE> : public QueueBridgeBase<msg::Stamped<std::shared_ptr<msg::PointCloud>>, msg::PointCloudStamped, FORCE>
{
public:
    using QueueBridgeBase<msg::Stamped<std::shared_ptr<msg::PointCloud>>, msg::PointCloudStamped, FORCE>::QueueBridgeBase;
    ~QueueBridge() override = default;

protected:
    void send(const msg::Stamped<std::shared_ptr<msg::PointCloud>>& val) override
    {
        msg::PointCloud pcl = *val.data_;
        this->sender_.send(msg::PointCloudStamped{std::move(pcl), val.timestamp_});
    }
};

template<typename T, bool FORCE>
class QueueBridge<msg::Stamped<T>, FORCE> : public QueueBridgeBase<msg::Stamped<T>, msg::Stamped<T>, FORCE>
{
public:
    using QueueBridgeBase<msg::Stamped<T>, msg::Stamped<T>, FORCE>::QueueBridgeBase;
    ~QueueBridge() override = default;

protected:
    void send(const msg::Stamped<T>& val) override
    {
        this->sender_.send(val);
    }
};

} // namespace fastsense::comm