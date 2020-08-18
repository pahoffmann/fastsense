//
// Created by julian on 8/17/20.
//

#include "imu_msg.h"

ImuMsg::ImuMsg(const double* acceleration, const double* angular_rate) : acc(acceleration), ang(angular_rate)
{}

ImuMsg::ImuMsg() : acc(), ang() {}

std::ostream& operator<<(std::ostream& os, const ImuMsg& data)
{
    os << "-- acc --\n";
    os << data.acc.x() << "\n";
    os << data.acc.y() << "\n";
    os << data.acc.z() << "\n";

    os << "-- ang --\n";
    os << data.ang.x() << "\n";
    os << data.ang.y() << "\n";
    os << data.ang.z() << "\n";
    return os;
}
