#pragma once

template<typename T>
ConcurrentRingBuffer<T>::ConcurrentRingBuffer(size_t size) :
    buffer(size),
    length(0),
    pushIdx(0),
    popIdx(0)
{
}

template<typename T>
bool ConcurrentRingBuffer<T>::push_nb(const T& val, bool force)
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

template<typename T>
void ConcurrentRingBuffer<T>::push(const T&)
{
    std::unique_lock<std::mutex> lock(mutex);
    if (length == buffer.size())
    {
        cvFull.wait(lock, [&] { return length < buffer.size(); });
    }

    doPush();
    cvEmpty.notify_one();
}

template<typename T>
bool ConcurrentRingBuffer<T>::pop_nb(T* val)
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

template<typename T>
void ConcurrentRingBuffer<T>::pop(T* val)
{
    std::unique_lock<std::mutex> lock(mutex);
    if (length == 0)
    {
        cvEmpty.wait(lock, [&] { return length != 0; });
    }

    doPop(val);
    cvFull.notify_one();
}

template<typename T>
void ConcurrentRingBuffer<T>::clear()
{
    std::unique_lock<std::mutex> lock(mutex);
    std::fill(buffer.begin(), buffer.end(), T());
    length = 0;
    pushIdx = 0;
    popIdx = 0;
    cvFull.notify_all();
}

template<typename T>
void ConcurrentRingBuffer<T>::doPush(const T& val)
{
    buffer[pushIdx] = val;
    length++;
    pushIdx++;
    if (pushIdx == buffer.size())
    {
        pushIdx = 0;
    }
}

template<typename T>
void ConcurrentRingBuffer<T>::doPop(T* val)
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
