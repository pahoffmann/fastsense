/**
 * @file buffer.h
 * @author Julian Gaal, Marcel Flottmann
 * @date 2020-09-09
 */

#pragma once

#include <CL/cl2.hpp>
#include <hw/types.h>
#include <memory>

// because xilinx missed this: Host -> Device, CL_MIGRATE_MEM_OBJECT_HOST defined in cl.h
#define CL_MIGRATE_MEM_OBJECT_DEVICE                  (0 << 0)

namespace fastsense::buffer {

// TODO stl container

/**
 * @brief Buffer class: a safe wrapper around cl::Buffer that allocates memory, 
 * maps this to a virtual address, and deallocates memory in destructor
 * 
 * @tparam T type of single element in buffer
 */
template <typename T>
class Buffer
{
private:
    /**
     * @brief unmap buffer: deattach virtual address from buffer
     */
    void unmapMemory() 
    {
        queue_->enqueueUnmapMemObject(buffer_, virtual_address_);
        queue_->finish();
    }

    /**
     * @brief map buffer: attach virtual address from buffer
     */
    void mapMemory() 
    {
        virtual_address_ = static_cast<T*>(queue_->enqueueMapBuffer(buffer_, CL_TRUE, map_flag_, 0, size_in_bytes_));
    }

protected:
    /// Xilinx command queue
    CommandQueuePtr queue_;

    /// number of elements in buffer/ size
    size_t num_elements_;
    
    /// size of buffer, in bytes
    size_t size_in_bytes_;
    
    /// underlying cl::Buffer
    cl::Buffer buffer_;
    
    /// mem flag: read or write buffer
    cl_mem_flags mem_flag_;
    
    /// map flag: read or write virtual address buffer
    cl_map_flags map_flag_;
    
    /// virtual address, to which buffer is mapped
    T* virtual_address_;

    /**
     * @brief Construct a new Buffer object
     * 
     * @param queue program command queue
     * @param context program context
     * @param num_elements number of elements in buffer
     * @param mem_flag sets type of buffer: read or write
     * @param map_flag sets type of virtual address buffer: read or write
     */
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
    /**
     * @brief Destroy the Buffer object and unmap memory
     */
    ~Buffer()
    {   
        unmapMemory();
    }

    /**
     * @brief delete assignment operator because of pointer member variable
     * 
     * @return Buffer& other Buffer
     */
    Buffer& operator=(Buffer&) = delete;

    /**
     * @brief delete copy constructor because of pointer member variable
     * @param Buffer& other buffer
     */
    Buffer(Buffer&) = delete;

    /**
     * @brief Get the virtual address that buffer was mapped to
     * 
     * @return T* address to virtal address
     */
    T* getVirtualAddress() 
    { 
        return virtual_address_; 
    }

    /**
     * @brief Get the Buffer object
     * 
     * @return cl::Buffer& 
     */
    cl::Buffer& getBuffer() 
    { 
        return buffer_; 
    }
    
    /**
     * @brief Return number of elements in buffer
     * 
     * @return size_t number or elements in buffer
     */
    size_t size() 
    { 
        return num_elements_; 
    }
    
    /**
     * @brief Return the size of the buffer in bytes
     * 
     * @return size_t size in bytes
     */
    size_t sizeInBytes() 
    { 
        return sizeof(T) * num_elements_; 
    }
};

/**
 * @brief Read only buffer
 * 
 * @tparam T type of single element in buffer
 */
template <typename T>
class InputBuffer : public Buffer<T>
{
public:
    /**
     * @brief Construct a new Input Buffer object
     * 
     * @param queue 
     * @param context 
     * @param num_elements 
     */
    InputBuffer(CommandQueuePtr queue, cl::Context& context, size_t num_elements)
    :   Buffer<T>(queue, context, num_elements, CL_MEM_READ_ONLY, CL_MAP_WRITE) 
    {}

    ///
    ~InputBuffer() = default;

    InputBuffer& operator=(InputBuffer&) = delete;
    InputBuffer(InputBuffer&) = delete;
};

/**
 * @brief Write only buffer
 * 
 * @tparam T type of single element in buffer
 */
template <typename T>
class OutputBuffer : public Buffer<T>
{
public:
    OutputBuffer(CommandQueuePtr queue, cl::Context& context, size_t num_elements)
    :   Buffer<T>(queue, context, num_elements, CL_MEM_WRITE_ONLY, CL_MAP_READ) 
    {}

    ~OutputBuffer() = default;

    OutputBuffer& operator=(OutputBuffer&) = delete;
    OutputBuffer(OutputBuffer&) = delete;
};

} // namespace fastsense::buffer