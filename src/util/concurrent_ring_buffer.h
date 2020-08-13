/**
 * @author Marcel Flottmann
 * @date   2020-08-11
 */

#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>

template<typename T>
class concurrent_ring_buffer
{
public:
    explicit concurrent_ring_buffer(size_t size) :
        buffer(size),
        length(0),
        push_idx(0),
        pop_idx(0)
    {
    }

    bool push_nb(const T& val, bool force = false)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if(length == buffer.size())
        {
            if(force)
            {
                do_pop(nullptr);
            }
            else
            {
                return false;
            }
        }

        do_push(val);
        cv_empty.notify_one();
        return true;
    }

    void push(const T&)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if(length == buffer.size())
        {
            cv_full.wait(lock, [&]{ return length < buffer.size(); });
        }

        do_push();
        cv_empty.notify_one();
    }

    bool pop_nb(T* val)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if(length == 0)
        {
            return false;
        }

        do_pop(val);
        cv_full.notify_one();
        return true;
    }

    void pop(T* val)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if(length == 0)
        {
            cv_empty.wait(lock, [&]{ return length != 0; });
        }

        do_pop(val);
        cv_full.notify_one();
    }

    void clear()
    {
        std::unique_lock<std::mutex> lock(mutex);
        std::fill(buffer.begin(), buffer.end(), T());
        length = 0;
        push_idx = 0;
        pop_idx = 0;
        cv_full.notify_all();
    }

    using ptr = std::shared_ptr<concurrent_ring_buffer<T>>;

private:
    void do_push(const T& val)
    {
        buffer[push_idx] = val;
        length++;
        push_idx++;
        if(push_idx == buffer.size())
        {
            push_idx = 0;
        }
    }

    void do_pop(T* val)
    {
        if(val != nullptr)
        {
            *val = buffer[pop_idx];
        }
        length--;
        pop_idx++;
        if(pop_idx == buffer.size())
        {
            pop_idx = 0;
        }
    }

    std::vector<T> buffer;
    size_t length;
    size_t push_idx;
    size_t pop_idx;
    std::mutex mutex;
    std::condition_variable cv_empty;
    std::condition_variable cv_full;
};
