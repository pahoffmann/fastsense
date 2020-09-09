#pragma once

#include <CL/cl2.hpp>
#include "base_kernel.h"

namespace fastsense::util {

class VaddKernel : public BaseKernel
{
private:
    cl::Buffer buffer_a;
    cl::Buffer buffer_b;
    cl::Buffer buffer_result;
public:
    VaddKernel(const cl::Context& context, const cl::Program& program, char* name, size_t size_in_bytes);
    ~VaddKernel() = default;
};

} // namespace fastsense::util