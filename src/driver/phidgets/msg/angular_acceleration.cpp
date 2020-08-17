//
// Created by julian on 8/17/20.
//

#include "angular_acceleration.h"
#include <cmath>

AngularRate::AngularRate(const double *angular_rate) : XYZ() {
    data_[0] = angular_rate[0] * (M_PI / 180.0);
    data_[1] = angular_rate[1] * (M_PI / 180.0);
    data_[2] = angular_rate[2] * (M_PI / 180.0);
}
