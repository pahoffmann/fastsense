#pragma once

/**
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <memory>

#include <hw/opencl.h>

// TODO sensor sync stuff here, too?
namespace fastsense
{
using CommandQueuePtr = std::shared_ptr<cl::CommandQueue>;
}