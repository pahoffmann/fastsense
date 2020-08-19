//
// Created by julian on 8/17/20.
//

#ifndef SRC_ANGULAR_ACCELERATION_H
#define SRC_ANGULAR_ACCELERATION_H

#include "../params.h"
#include "xyz.h"

class AngularVelocity : public XYZ {
public:
    AngularVelocity() = default;
    AngularVelocity(const double* angular_rate);
};


#endif //SRC_ANGULAR_ACCELERATION_H
