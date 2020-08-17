//
// Created by julian on 8/17/20.
//

#include <string.h>
#include "three_dim.h"

ThreeDim::ThreeDim(const double *data) : data_() {
    memcpy((void *)&data_, &data, sizeof(3 * sizeof(double)));
}

const double &ThreeDim::x() const {
    return data_[0];
}

const double &ThreeDim::y() const {
    return data_[1];
}

const double &ThreeDim::z() const {
    return data_[2];
}
