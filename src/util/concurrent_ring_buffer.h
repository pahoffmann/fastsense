/**
 * @author Marcel Flottmann
 * @date   2020-08-11
 */

#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>

template<typename T>
class ConcurrentRingBuffer
{
public:
    explicit ConcurrentRingBuffer(size_t size) :
        buffer(size),
        length(0),
        pushIdx(0),
        popIdx(0)
    {
    }

    bool push_nb(const T& val, bool force = false)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (length == buffer.size())
        {
            if (force)
            {
                doPop(nullptr);
            }
            else
            {
                return false;
            }
        }

        doPush(val);
        cvEmpty.notify_one();
        return true;
    }

    void push(const T&)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (length == buffer.size())
        {
            cvFull.wait(lock, [&] { return length < buffer.size(); });
        }

        doPush();
        cvEmpty.notify_one();
    }

    bool pop_nb(T* val)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (length == 0)
        {
            return false;
        }

        doPop(val);
        cvFull.notify_one();
        return true;
    }

    void pop(T* val)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (length == 0)
        {
            cvEmpty.wait(lock, [&] { return length != 0; });
        }

        doPop(val);
        cvFull.notify_one();
    }

    void clear()
    {
        std::unique_lock<std::mutex> lock(mutex);
        std::fill(buffer.begin(), buffer.end(), T());
        length = 0;
        pushIdx = 0;
        popIdx = 0;
        cvFull.notify_all();
    }

    using ptr = std::shared_ptr<ConcurrentRingBuffer<T>>;

private:
    void doPush(const T& val)
    {
        buffer[pushIdx] = val;
        length++;
        pushIdx++;
        if (pushIdx == buffer.size())
        {
            pushIdx = 0;
        }
    }

    void doPop(T* val)
    {
        if (val != nullptr)
        {
            *val = buffer[popIdx];
        }
        length--;
        popIdx++;
        if (popIdx == buffer.size())
        {
            popIdx = 0;
        }
    }

    std::vector<T> buffer;
    size_t length;
    size_t pushIdx;
    size_t popIdx;
    std::mutex mutex;
    std::condition_variable cvEmpty;
    std::condition_variable cvFull;
};
