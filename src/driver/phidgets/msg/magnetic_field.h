//
// Created by julian on 8/18/20.
//

#ifndef SRC_MAGNETIC_FIELD_H
#define SRC_MAGNETIC_FIELD_H

#include "xyz.h"

class MagneticField : public XYZ {
public:
    MagneticField() = default;
    explicit MagneticField(const double* magneticField);
};


#endif //SRC_MAGNETIC_FIELD_H
