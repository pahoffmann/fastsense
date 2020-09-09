#pragma once

#include <CL/cl2.hpp>
#include <memory>


// TODO sensor sync stuff here, too?
namespace fastsense {
    using CommandQueuePtr = std::shared_ptr<cl::CommandQueue>;
}