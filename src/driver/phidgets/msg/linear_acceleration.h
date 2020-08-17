//
// Created by julian on 8/17/20.
//

#ifndef SRC_LINEAR_ACCELERATION_H
#define SRC_LINEAR_ACCELERATION_H

#include "xyz.h"

class Acceleration : public XYZ {
public:
    explicit Acceleration(const double* acceleration);
};


#endif //SRC_LINEAR_ACCELERATION_H
