/**
 * @file event_handler_list.h
 * @author Marcel Flottmann
 * @date 2020-09-05
 */

#pragma once

#include <functional>
#include <list>
#include <mutex>

template<typename T>
class EventHandlerList
{
    std::list<std::function<T>> callbacks;
    std::mutex mtx;
public:
    using handle_t = typename std::list<std::function<T>>::iterator;

    handle_t add(const std::function<T>& callback)
    {
        std::lock_guard lock(mtx);
        return callbacks.insert(callbacks.end(), callback);
    }

    void remove(const handle_t& handle)
    {
        std::lock_guard lock(mtx);
        callbacks.erase(handle);
    }

    template<typename ...Args>
    void invoke(Args... args)
    {
        for (auto& handler : callbacks)
        {
            handler(args...);
        }
    }
};
