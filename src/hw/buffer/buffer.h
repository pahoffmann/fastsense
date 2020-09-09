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
    void unmapMemory();

    /**
     * @brief map buffer: attach virtual address from buffer
     */
    void mapMemory();

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
           cl_map_flags map_flag);

public:
    using value_type = T;
    using reference = T&;
    using const_reference = const T&;
    using difference_type = ssize_t;
    using size_type = size_t;
    /**
     * @brief Destroy the Buffer object and unmap memory
     */
    ~Buffer();

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
    T* getVirtualAddress();

    /**
     * @brief Get the Buffer object
     * 
     * @return cl::Buffer& 
     */
    cl::Buffer& getBuffer();
    
    /**
     * @brief Return number of elements in buffer
     * 
     * @return size_t number or elements in buffer
     */
    size_t size();
    
    /**
     * @brief Return the size of the buffer in bytes
     * 
     * @return size_t size in bytes
     */
    size_t sizeInBytes();

    // class iterator {
    // public:
    //     using value_type = T;
    //     using reference = T&;
    //     using const_reference = const T&;
    //     using difference_type = ssize_t;
    //     using size_type = size_t;
    //     using pointer = T*;

    //     iterator();
    //     iterator(const fastsense::buffer::Buffer<T>::const_iterator&);
    //     iterator(const iterator&);
    //     ~iterator();

    //     iterator& operator=(const iterator&);
    //     bool operator==(const iterator&) const;
    //     bool operator!=(const iterator&) const;
    //     bool operator<(const iterator&) const;
    //     bool operator>(const iterator&) const;
    //     bool operator<=(const iterator&) const;
    //     bool operator>=(const iterator&) const;

    //     iterator& operator++();
    //     iterator& operator--();
    //     iterator& operator+=(size_type);
    //     iterator operator+(size_type) const;
    //     iterator& operator-=(size_type);            
    //     iterator operator-(size_type) const;
    //     difference_type operator-(iterator) const;

    //     reference operator*() const;
    //     pointer operator->() const;
    //     reference operator[](size_type) const;
    // };

    // class const_iterator {
    // public:
    //     using value_type = T;
    //     using reference = T&;
    //     using const_reference = const T&;
    //     using difference_type = ssize_t;
    //     using size_type = size_t;
    //     using pointer = T*;

    //     const_iterator();
    //     const_iterator(const const_iterator&);
    //     const_iterator(const iterator&);
    //     ~const_iterator();

    //     const_iterator& operator=(const const_iterator&);
    //     bool operator==(const const_iterator&) const;
    //     bool operator!=(const const_iterator&) const;
    //     bool operator<(const const_iterator&) const;
    //     bool operator>(const const_iterator&) const;
    //     bool operator<=(const const_iterator&) const;
    //     bool operator>=(const const_iterator&) const;

    //     const_iterator& operator++();
    //     const_iterator& operator--();
    //     const_iterator& operator+=(size_type);
    //     const_iterator operator+(size_type) const;
    //     const_iterator& operator-=(size_type);            
    //     const_iterator operator-(size_type) const;
    //     difference_type operator-(const_iterator) const;

    //     reference operator*() const;
    //     pointer operator->() const;
    //     reference operator[](size_type) const;
    // };
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

#include "buffer.tcc"