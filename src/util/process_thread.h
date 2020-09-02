//
// Created by julian on 9/2/20.
//

#pragma once

#include <thread>

namespace fastsense::util
{

class ProcessThread
{
public:
    ProcessThread();

    virtual ~ProcessThread() = default;

    virtual void start() = 0;

    virtual void stop() = 0;

protected:
    /// Worker thread
    std::thread worker;
    /// Flag if the thread is running
    bool running;
};

}
