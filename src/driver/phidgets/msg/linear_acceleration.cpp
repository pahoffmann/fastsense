/**
 * @file linear_acceleration.cpp
 * @author Julian Gaal
 * @date 2020-08-18
 */

#include "linear_acceleration.h"
#include "../params.h"

using namespace fastsense::driver::msg;

LinearAcceleration::LinearAcceleration(const double* acceleration) : XYZ()
{
    data_[0] = -acceleration[0] * fastsense::driver::params::G;
    data_[1] = -acceleration[1] * fastsense::driver::params::G;
    data_[2] = -acceleration[2] * fastsense::driver::params::G;
}
