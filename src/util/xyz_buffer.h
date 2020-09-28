/**
 * @file xyz_buffer.h
 * @author Julian Gaal
 * @date 2020-08-17
 */

#pragma once

#include <cassert>

// TODO operator * - + / = 
// TODO length of buffer

namespace fastsense::util
{

/**
 * @brief represents a data point with x, y and z "coordinates"
 */
template <typename T>
class XYZBuffer
{
public:
    explicit XYZBuffer() = default;

    XYZBuffer(T val) : data_{val, val, val} {}

    XYZBuffer(T valX, T valY, T valZ) : data_{valX, valY, valZ} {}

    virtual ~XYZBuffer() = default;

    inline const T& x() const
    {
        return data_[0];
    }

    inline const T& y() const
    {
        return data_[1];
    }

    inline const T& z() const
    {
        return data_[2];
    }

    inline T at(size_t index) const 
    {
        assert(index <= 2);

        return data_[index];
    }

    inline T* getData() const
    {
        return data_;
    }

protected:
    /// actual data
    T data_[3];
};


using LocalMapSize = util::XYZBuffer<int>;
using LocalMapPos = util::XYZBuffer<int>;
using LocalMapOffset = util::XYZBuffer<int>;

} // namespace driver::util
