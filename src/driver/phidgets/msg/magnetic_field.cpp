//
// Created by julian on 8/18/20.
//

#include <limits>
#include "magnetic_field.h"
#include "phidget21.h"

MagneticField::MagneticField(const double *magneticField) : XYZ() {
    if (magneticField[0] != PUNK_DBL)
    {
        // device reports data in Gauss, multiply by 1e-4 to convert to Tesla
        data_[0] = magneticField[0] * 1e-4;
        data_[1] = magneticField[1] * 1e-4;
        data_[2] = magneticField[2] * 1e-4;
    }
    else
    {
        double nan = std::numeric_limits<double>::quiet_NaN();

        data_[0] = nan;
        data_[1] = nan;
        data_[2] = nan;
    }
}