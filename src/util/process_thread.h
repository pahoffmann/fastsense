#pragma once

/**
 * @file process_thread.h
 * @author Julian Gaal
 * @author Malte Hillmann
 * @author Juri Vana
 */

#include <thread>

namespace fastsense::util
{

/**
 * @brief Abstract base process thread class.
 */
class ProcessThread
{
public:
    using UPtr = std::unique_ptr<ProcessThread>;

    /**
     * @brief Constructor of ProcessThread.
     */
    ProcessThread() : worker{}, running{false} {}

    /**
     * @brief Virtual destructor because this class will be inherited.
     */
    virtual ~ProcessThread()
    {
        stop();
    }
    
    /// Delete copy constructor.
    ProcessThread(ProcessThread&) = delete;

    /// Delete move constructor.
    ProcessThread(ProcessThread&&) = delete;

    /// Delete copy assignment operator.
    ProcessThread& operator=(const ProcessThread&) = delete;

    /// Delete move assignment operator.
    ProcessThread& operator=(ProcessThread&&) = delete;

    /**
     * @brief Starts the thread.
     */
    virtual void start()
    {
        if (!running)
        {
            running = true;
            worker = std::thread([&]()
            {
                this->thread_run();
            });
        }
    }

    /**
     * @brief Stops the thread if it is running.
     */
    virtual void stop()
    {
        if (running && worker.joinable())
        {
            running = false;
            worker.join();
        }
    }

protected:
    /**
     * @brief Pure virtual function that is executed after the thread is started.
     */
    virtual void thread_run() = 0;

    /// Worker thread
    std::thread worker;

    /// Flag if the thread is running
    bool running;
};

} // namespace fastsense::util
