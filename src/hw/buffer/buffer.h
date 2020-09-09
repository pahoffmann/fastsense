#pragma once

#include <CL/cl2.hpp>
#include <hw/types.h>
#include <memory>

// because xilinx missed this: Host -> Device, CL_MIGRATE_MEM_OBJECT_HOST defined in cl.h
#define CL_MIGRATE_MEM_OBJECT_DEVICE                  (0 << 0)

namespace fastsense::buffer {

// TODO stl container
// TODO nullptr

template <typename T>
class Buffer
{
private:
    void unmapMemory() 
    {
        queue_->enqueueUnmapMemObject(buffer_, virtual_address_);
        queue_->finish();
    }

    void mapMemory() 
    {
        virtual_address_ = static_cast<T*>(queue_->enqueueMapBuffer(buffer_, CL_TRUE, map_flag_, 0, size_in_bytes_));
    }

protected:
    CommandQueuePtr queue_;
    size_t num_elements_;
    size_t size_in_bytes_;
    cl::Buffer buffer_;
    cl_mem_flags mem_flag_;
    cl_map_flags map_flag_;
    T* virtual_address_;

    Buffer(CommandQueuePtr queue, 
           cl::Context& context,
           size_t num_elements, 
           cl_mem_flags mem_flag, 
           cl_map_flags map_flag) 
        : queue_(queue), 
          num_elements_(num_elements), 
          size_in_bytes_(sizeof(T)*num_elements),
          buffer_(context, mem_flag, size_in_bytes_),
          mem_flag_(mem_flag),
          map_flag_(map_flag),
          virtual_address_(nullptr)
    {
        mapMemory();
    }

public:
    ~Buffer()
    {   
        unmapMemory();
    }

    T* getVirtualAddress() { return virtual_address_; }
    cl::Buffer& getBuffer() { return buffer_; }
    size_t size() { return num_elements_; }
    size_t sizeInBytes() { return sizeof(T) *num_elements_; }
};

template <typename T>
class InputBuffer : public Buffer<T>
{
public:
    InputBuffer(CommandQueuePtr queue, cl::Context& context, size_t num_elements)
    :   Buffer<T>(queue, context, num_elements, CL_MEM_READ_ONLY, CL_MAP_WRITE) 
    {}

    ~InputBuffer() = default;
};

template <typename T>
class OutputBuffer : public Buffer<T>
{
public:
    OutputBuffer(CommandQueuePtr queue, cl::Context& context, size_t num_elements)
    :   Buffer<T>(queue, context, num_elements, CL_MEM_WRITE_ONLY, CL_MAP_READ) 
    {}

    ~OutputBuffer() = default;
};

} // namespace fastsense::buffer