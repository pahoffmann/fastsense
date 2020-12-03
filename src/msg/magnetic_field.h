#pragma once

/**
 * @file magnetic_field.h
 * @author Julian Gaal
 */

#include <util/point.h>
#include <limits>

// Taken from phidget21.h header, for portability
#define PUNK_DBL    1e300                   /**< Unknown Double */

namespace fastsense::msg
{

/**
 * @brief Represents magnetic field data from imu
 */
struct MagneticField : public Vector3f
{
    MagneticField()
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {}

    MagneticField(float x, float y, float z)
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {
        (*this)[0] = x;
        (*this)[1] = y;
        (*this)[2] = z;
    }

    explicit MagneticField(const double* magneticField)
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {
        if (magneticField[0] != PUNK_DBL)
        {
            // device reports data in Gauss, multiply by 1e-4 to convert to Tesla
            (*this)[0] = magneticField[0] * 1e-4;
            (*this)[1] = magneticField[1] * 1e-4;
            (*this)[2] = magneticField[2] * 1e-4;
        }
        else
        {
            double nan = std::numeric_limits<double>::quiet_NaN();

            (*this)[0] = nan;
            (*this)[1] = nan;
            (*this)[2] = nan;
        }
    }
};

} // namespace fastsense::msg