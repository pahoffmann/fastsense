/**
 * @file xyz_buffer.h
 * @author Julian Gaal
 * @date 2020-08-17
 */

#pragma once

#include <stdexcept>

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
    XYZBuffer() = default;

    explicit XYZBuffer(T val) : data_{val, val, val} {}

    XYZBuffer(T valX, T valY, T valZ) : data_{valX, valY, valZ} {}

    virtual ~XYZBuffer() = default;

    inline const T& x() const
    {
        return data_[0];
    }

    inline T& x()
    {
        return data_[0];
    }

    inline const T& y() const
    {
        return data_[1];
    }

    inline T& y()
    {
        return data_[1];
    }

    inline const T& z() const
    {
        return data_[2];
    }

    inline T& z()
    {
        return data_[2];
    }

    inline T at(size_t index) const 
    {
        if (index > 2)
        {
            throw std::out_of_range("Can't access xyz buffer at " + index);
        }

        return data_[index];
    }

    inline const T* getData() const
    {
        return data_;
    }

    XYZBuffer<T>& operator+=(const T& rhs)
    {
      x() += rhs;
      y() += rhs;
      z() += rhs;
      return *this;
    }

    XYZBuffer<T>& operator+=(const XYZBuffer<T>& rhs)
    {
      x() += rhs.x();
      y() += rhs.y();
      z() += rhs.z();
      return *this;
    }

protected:
    /// actual data
    T data_[3];
};


using LocalMapSize = util::XYZBuffer<int>;
using LocalMapPos = util::XYZBuffer<int>;
using LocalMapOffset = util::XYZBuffer<int>;

} // namespace driver::util
