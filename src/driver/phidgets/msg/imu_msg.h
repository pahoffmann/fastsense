//
// Created by julian on 8/17/20.
//

#ifndef SRC_IMU_MSG_H
#define SRC_IMU_MSG_H

#include "../params.h"
#include "linear_acceleration.h"
#include "angular_velocity.h"
#include "magnetic_field.h"
#include <iostream>
#include <cmath>
#include <memory>

class ImuMsg {
public:
    ImuMsg();
    ImuMsg(const double* acc, const double* ang, const double* magField);
    LinearAcceleration acc;
    AngularVelocity ang;
    MagneticField mag;
};

std::ostream& operator<<(std::ostream& os, const ImuMsg& data);

#endif //SRC_IMU_MSG_H
