//
// Created by julian on 8/17/20.
//

#include "linear_acceleration.h"
#include "../params.h"

LinearAcceleration::LinearAcceleration(const double *acceleration) : XYZ() {
    data_[0] = -acceleration[0] * phidgets::params::G;
    data_[1] = -acceleration[1] * phidgets::params::G;
    data_[2] = -acceleration[2] * phidgets::params::G;
}
