//
// Created by julian on 8/17/20.
//

#ifndef SRC_IMU_DATA_H
#define SRC_IMU_DATA_H

#include "params.h"
#include <iostream>
#include <cmath>

struct ImuData {
public:
    ImuData(const double* acceleration, const double* angular_rate);

private:
    double acceleration_[3];
    double angularRate_[3];

    void set_acceleration(const double *acceleration) {
        acceleration_[0] = -acceleration[0] * phidgets::params::G;
        acceleration_[1] = -acceleration[1] * phidgets::params::G;
        acceleration_[2] = -acceleration[2] * phidgets::params::G;
    }

    void set_angular_rate(const double *angular_rate) {
        angularRate_[0] = angular_rate[0] * (M_PI / 180.0);
        angularRate_[1] = angular_rate[1] * (M_PI / 180.0);
        angularRate_[2] = angular_rate[2] * (M_PI / 180.0);
    }
};

std::ostream& operator<<(std::ostream& os, const ImuData& data)
{
    os << "-- acc --\n";
    os << -data.acceleration[0] * phidgets::params::G << "\n";
    os << -data.acceleration[1] * phidgets::params::G << "\n";
    os << -data.acceleration[2] * phidgets::params::G << "\n";

    os << "-- ang --\n";
    os << data.angularRate[0] * (M_PI / 180.0) << "\n";
    os << data.angularRate[1] * (M_PI / 180.0) << "\n";
    os << data.angularRate[2] * (M_PI / 180.0) << "\n";
    return os;
}

#endif //SRC_IMU_DATA_H
