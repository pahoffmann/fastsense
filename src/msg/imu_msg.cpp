/**
 * @file imu_msg.cpp
 * @author Julian Gaal
 * @date 2020-08-18
 */

#include <msg/imu_msg.h>

using namespace fastsense::msg;

ImuMsg::ImuMsg(const double* acceleration, const double* angular_rate, const double* magField) : acc(acceleration), ang(angular_rate), mag(magField)
{}

ImuMsg::ImuMsg() : acc(), ang(), mag() {}

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

    os << "-- mag --\n";
    os << data.mag.x() << "\n";
    os << data.mag.y() << "\n";
    os << data.mag.z() << "\n";

    return os;
}
