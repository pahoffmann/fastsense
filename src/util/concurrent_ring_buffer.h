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
    explicit ConcurrentRingBuffer(size_t size);

    bool push_nb(const T& val, bool force = false);

    void push(const T&);

    bool pop_nb(T* val);

    void pop(T* val);

    void clear();

    using ptr = std::shared_ptr<ConcurrentRingBuffer<T>>;

private:
    void doPush(const T& val);

    void doPop(T* val);

    std::vector<T> buffer;
    size_t length;
    size_t pushIdx;
    size_t popIdx;
    std::mutex mutex;
    std::condition_variable cvEmpty;
    std::condition_variable cvFull;
};

#include "concurrent_ring_buffer.tcc"
