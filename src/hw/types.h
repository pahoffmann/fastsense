/**
 * @file types.h
 * @author Julian Gaal, Marcel Flottmann
 * @date 2020-09-12
 */
#pragma once

#include <hw/opencl.h>
#include <memory>


// TODO sensor sync stuff here, too?
namespace fastsense
{
using CommandQueuePtr = std::shared_ptr<cl::CommandQueue>;
}