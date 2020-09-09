#include "kernel_vadd.h"

using namespace fastsense::kernels;

VaddKernel::VaddKernel(const cl::Context& context, const cl::Program& program, char* name, size_t size_in_bytes) 
    : BaseKernel(program, name), 
      buffer_a(context, CL_MEM_READ_ONLY, size_in_bytes), 
      buffer_b(context, CL_MEM_READ_ONLY, size_in_bytes), 
      buffer_result(context, CL_MEM_WRITE_ONLY, size_in_bytes)
{
    setArgs(buffer_a, buffer_b, buffer_result);
    setArg(4096);
}