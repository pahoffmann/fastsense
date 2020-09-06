/**
 * @file linear_acceleration.cpp
 * @author Julian Gaal
 * @date 2020-08-18
 */

#include <msg/linear_acceleration.h>
#include <util/params.h>

using namespace fastsense::msg;

LinearAcceleration::LinearAcceleration(const double* acceleration) : XYZBuffer<double>()
{
    data_[0] = -acceleration[0] * fastsense::util::params::G;
    data_[1] = -acceleration[1] * fastsense::util::params::G;
    data_[2] = -acceleration[2] * fastsense::util::params::G;
}
