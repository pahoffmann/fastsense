#pragma once

/**
  * @file button_reader.h
  * @author julian 
  * @date 1/21/21
 */

#include "process_thread.h"
#include "logging/logger.h"

#include <iostream>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

namespace fastsense::util
{

class ButtonReader : public ProcessThread
{
public:
    ButtonReader(const char* filepath, std::chrono::milliseconds timeout)
    : fd{open(filepath, O_RDONLY)}
    , pfd{fd, POLLIN, 0}
    , timeout{timeout}
    {
        if (fd == -1)
        {
            throw std::runtime_error("Can't open button press file for reading");
        }
    }

    ~ButtonReader() final
    {
        if (close(fd) == -1) {
            logging::Logger::error("Can't close button press");
        }
    }

    void thread_run() override
    {
        while (running)
        {
            if (read_file())
            {
                logging::Logger::info("Got button press");
            }
        }
    }

private:
    bool read_file()
    {
        int ret = 0;
        while(ret == 0)
        {
            ret = poll(&pfd, 1, timeout.count());  // timeout of 1000ms
            if(ret == 1)
            {
                return true;
            }
            else if(ret == -1)
            {
                std::cout << "Error: " << strerror(errno) << std::endl;
            }
        }

        return false;
    }

    int fd;
    struct pollfd pfd;
    std::chrono::milliseconds timeout;
};

} // namespace fastsense::util


