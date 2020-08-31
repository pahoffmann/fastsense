/**
 * @file angular_velocity.cpp
 * @author Julian Gaal
 * @date 2020-08-17
 */

#include <util/msg/angular_velocity.h>
#include <cmath>

using namespace fastsense::util::msg;

AngularVelocity::AngularVelocity(const double* angular_rate) : XYZ()
{
    data_[0] = angular_rate[0] * (M_PI / 180.0);
    data_[1] = angular_rate[1] * (M_PI / 180.0);
    data_[2] = angular_rate[2] * (M_PI / 180.0);
}
