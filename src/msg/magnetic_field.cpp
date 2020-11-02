/**
 * @author Julian Gaal
 */

#include <limits>
#include <phidget21.h>

#include <msg/magnetic_field.h>

// Taken from phidget21.h header, for portability
#define PUNK_DBL	1e300					/**< Unknown Double */

using namespace fastsense::msg;

MagneticField::MagneticField(const double* magneticField)
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