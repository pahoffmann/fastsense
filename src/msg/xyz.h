/**
 * @file xyz.h
 * @author Julian Gaal
 * @date 2020-08-17
 */

#pragma once

namespace fastsense::msg
{

/**
 * @brief represents a data point with x, y and z "coordinates"
 */
class XYZ
{
public:
    explicit XYZ() = default;

    inline const double& x() const
    {
        return data_[0];
    }

    inline const double& y() const
    {
        return data_[1];
    }

    inline const double& z() const
    {
        return data_[2];
    }

protected:
    /// actual data
    double data_[3];
};

} // namespace driver::msg
