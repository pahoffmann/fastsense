/**
 * @file buffer.tcc
 * @author Julian Gaal, Marcel Flottmann
 * @date 2020-09-09
 */

namespace fastsense::buffer {

template <typename T>
void Buffer<T>::unmapMemory() 
{
    queue_->enqueueUnmapMemObject(buffer_, virtual_address_);
    queue_->finish();
}

template <typename T>
void Buffer<T>::mapMemory() 
{
    virtual_address_ = static_cast<T*>(queue_->enqueueMapBuffer(buffer_, CL_TRUE, map_flag_, 0, size_in_bytes_));
}

template <typename T>
Buffer<T>::Buffer(CommandQueuePtr queue, 
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

/**
 * @brief Destroy the Buffer object and unmap memory
 */
template <typename T>
Buffer<T>::~Buffer()
{   
    unmapMemory();
}

template <typename T>
T* Buffer<T>::getVirtualAddress() 
{ 
    return virtual_address_; 
}

template <typename T>
cl::Buffer& Buffer<T>::getBuffer() 
{ 
    return buffer_; 
}

template <typename T>
size_t Buffer<T>::size() 
{ 
    return num_elements_; 
}

template <typename T>
size_t Buffer<T>::sizeInBytes() 
{ 
    return sizeof(T) * num_elements_; 
} 

// Buffer<T>::iterator::iterator()
// {

// }

// template <typename T>
// Buffer<T>::iterator::iterator(const Buffer<T>::const_iterator&)
// {

// }

// template <typename T>
// Buffer<T>::iterator::iterator(const Buffer<T>::iterator&)
// {

// }

// template <typename T>
// Buffer<T>::iterator::~iterator()
// {

// }

// template <typename T>
// Buffer<T>::iterator& Buffer<T>::iterator::operator=(const Buffer<T>::iterator&)
// {

// }

// template <typename T>
// bool Buffer<T>::iterator::operator==(const Buffer<T>::iterator&) const
// {

// }

// template <typename T>
// bool Buffer<T>::iterator::operator!=(const Buffer<T>::iterator&) const
// {

// }

// template <typename T>
// bool Buffer<T>::iterator::operator<(const Buffer<T>::iterator&) const
// {

// }

// template <typename T>
// bool Buffer<T>::iterator::operator>(const Buffer<T>::iterator&) const
// {

// }

// template <typename T>
// bool Buffer<T>::iterator::operator<=(const Buffer<T>::iterator&) const
// {

// }

// template <typename T>
// bool Buffer<T>::iterator::operator>=(const Buffer<T>::iterator&) const
// {

// }

// template <typename T>
// Buffer<T>::iterator& Buffer<T>::iterator::operator++()
// {

// }

// template <typename T>
// Buffer<T>::iterator& Buffer<T>::iterator::operator--()
// {

// }

// template <typename T>
// Buffer<T>::iterator&
// Buffer<T>::iterator::operator+=(Buffer<T>::iterator::size_type)
// {

// }

// template <typename T>
// Buffer<T>::iterator
// Buffer<T>::iterator::operator+(Buffer<T>::iterator::size_type) const
// {

// }

// template <typename T>
// Buffer<T>::iterator&
// Buffer<T>::iterator::operator-=(Buffer<T>::iterator::size_type)
// {

// }   

// template <typename T>
// Buffer<T>::iterator
// Buffer<T>::iterator::operator-(Buffer<T>::iterator::size_type) const
// {

// }

// template <typename T>
// Buffer<T>::iterator::difference_type
// Buffer<T>::iterator::operator-(Buffer<T>::iterator) const
// {

// }

// template <typename T>
// Buffer<T>::iterator::reference
// Buffer<T>::iterator::operator*() const
// {

// }

// template <typename T>
// Buffer<T>::iterator::pointer
// Buffer<T>::iterator::operator->() const
// {

// }

// template <typename T>
// Buffer<T>::iterator::reference
// Buffer<T>::iterator::operator[](Buffer<T>::iterator::size_type) const
// {

// }

// template <typename T>
// Buffer<T>::const_iterator::const_iterator() 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator::const_iterator(const Buffer<T>::const_iterator&) 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator::const_iterator(const Buffer<T>::iterator&) 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator::~const_iterator() 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator& operator=(const Buffer<T>::const_iterator&) 
// {

// }

// template <typename T>
// bool Buffer<T>::const_iterator::operator==(const Buffer<T>::const_iterator&) const 
// {

// }

// template <typename T>
// bool Buffer<T>::const_iterator::operator!=(const Buffer<T>::const_iterator&) const 
// {

// }

// template <typename T>
// bool Buffer<T>::const_iterator::operator<(const Buffer<T>::const_iterator&) const 
// {

// }

// template <typename T>
// bool Buffer<T>::const_iterator::operator>(const Buffer<T>::const_iterator&) const 
// {

// }

// template <typename T>
// bool Buffer<T>::const_iterator::operator<=(const Buffer<T>::const_iterator&) const 
// {

// }

// template <typename T>
// bool Buffer<T>::const_iterator::operator>=(const Buffer<T>::const_iterator&) const 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator::const_iterator&
// Buffer<t>::const_iterator::operator++() 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator&
// Buffer<t>::const_iterator::operator--() 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator&
// Buffer<t>::const_iterator::operator+=(Buffer<T>::iterator::size_type) 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator
// Buffer<t>::const_iterator::operator+(Buffer<T>::iterator::size_type) const 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator&
// Buffer<t>::const_iterator::operator-=(Buffer<T>::iterator::size_type) 
// {

// }   

// template <typename T>
// Buffer<T>::const_iterator
// Buffer<t>::const_iterator::operator-(Buffer<T>::iterator::size_type) const 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator::difference_type
// Buffer<t>::const_iterator::operator-(Buffer<T>::iterator::const_iterator) const 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator::reference
// Buffer<t>::const_iterator::operator*() const 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator::pointer
// Buffer<t>::const_iterator::operator->() const 
// {

// }

// template <typename T>
// Buffer<T>::const_iterator::reference
// Buffer<t>::const_iterator::operator[](Buffer<T>::iterator::size_type) const 
// {

// }

} // namespace fastsense::buffer