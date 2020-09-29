#pragma once

/**
 * @author Julian Gaal
 */

namespace fastsense::msg
{

/**
 * @brief represents a data point with x, y and z "coordinates"
 */
template <typename T>
class XYZBuffer
{
public:
    explicit XYZBuffer() = default;
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

protected:
    /// actual data
    T data_[3];
};

} // namespace driver::msg
