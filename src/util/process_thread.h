#pragma once

/**
 * @file process_thread.h
 * @author Julian Gaal
 */

#include <thread>

namespace fastsense::util
{

class ProcessThread
{
public:
    ProcessThread() : worker{}, running{false} {}

    virtual ~ProcessThread()
    {
        if (running && worker.joinable())
        {
            worker.join();
        }
    }

    virtual void start() = 0;

    virtual void stop() = 0;

protected:
    /// Worker thread
    std::thread worker;
    /// Flag if the thread is running
    bool running;
};

} // namespace fastsense::util
