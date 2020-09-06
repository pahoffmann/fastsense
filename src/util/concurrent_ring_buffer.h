/**
 * @file concurrent_ring_buffer.h
 * @author Marcel Flottmann
 * @date 2020-08-11
 */

#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>

namespace fastsense::util
{

/**
 * @brief Ring buffer with multithreading support.
 *
 * @tparam T Type of the buffer.
 */
template<typename T>
class ConcurrentRingBuffer
{
public:
    /**
     * @brief Construct a new Concurrent Ring Buffer object with specified size.
     *
     * @param size Size of the buffer.
     */
    explicit ConcurrentRingBuffer(size_t size);

    /**
     * @brief Push element non-blocking to the ring buffer.
     *
     * @param val Element to push.
     * @param force When true, the oldest element will be deleted if the buffer is full.
     * @return true Element was sucessfully pushed.
     * @return false The buffer is full.
     */
    bool push_nb(const T& val, bool force = false);

    /**
     * @brief Push element to the ring bufer. Blocks until an element is popped or the the buffer is cleared.
     *
     * @param val Element to push.
     */
    void push(const T& val);

    /**
     * @brief Pop element from the ring buffer
     *
     * @param val Pointer to element to fill with popped value. If val is a nullptr no value is assigned, but an element is popped nonetheless.
     * @return true Element popped successfully.
     * @return false The ring buffer is empty.
     */
    bool pop_nb(T* val);

    /**
     * @brief Pop element from the ring buffer. Blocks until an element is pushed.
     *
     * @param val Pointer to element to fill with popped value. If val is a nullptr no value is assigned, but an element is popped nonetheless.
     */
    void pop(T* val);

    /**
     * @brief Clear the buffer
     *
     */
    void clear();

    /**
     * @brief return buffer length
     * @return buffer length
     */
    inline size_t getLength() const
    {
        return length;
    }

    using ptr = std::shared_ptr<ConcurrentRingBuffer<T>>;

private:
    /**
     * @brief Actually do the push
     *
     * @param val Element to push.
     */
    void doPush(const T& val);

    /**
     * @brief Actually do the pop
     *
     * @param val Pointer to element to fill popped value. If val is a nullptr no value is assigned, but an element is popped nonetheless.
     */
    void doPop(T* val);

    /// Buffer containing the data
    std::vector<T> buffer;

    /// Current length of the bufffer
    size_t length;

    /// Current push position
    size_t pushIdx;

    /// Current pop position
    size_t popIdx;

    /// Mutex for locking
    std::mutex mutex;

    /// Condition variable to wait on, if the ring buffer is empty
    std::condition_variable cvEmpty;
    /// Condition variable to wait on, if the ring bufer is full
    std::condition_variable cvFull;
};

} // namespace fastsense::util

#include "concurrent_ring_buffer.tcc"
