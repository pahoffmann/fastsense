//
// Created by julian on 8/12/20.
//

#include "phidgets_imu.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;

// TODO detach handler? How to handle connected/disconnectedness

void
phidgets::PhidgetsImu::dataHandler(const double *acceleration, const double *angularRate, const double *magneticField,
                                   double timestamp) {
    std::cout << "===========\n-- acc --\n";
    std::cout << -acceleration[0] * G << "\n";
    std::cout << -acceleration[1] * G << "\n";
    std::cout << -acceleration[2] * G << "\n";

    std::cout << "-- ang --\n";
    std::cout << angularRate[0] * (M_PI / 180.0) << "\n";
    std::cout << angularRate[1] * (M_PI / 180.0) << "\n";
    std::cout << angularRate[2] * (M_PI / 180.0) << "\n";

    if (magneticField[0] != PUNK_DBL)
    {
        // device reports data in Gauss, multiply by 1e-4 to convert to Tesla
        std::cout << "-- mag -- \n";
        std::cout << magneticField[0] * 1e-4 << "\n";
        std::cout << magneticField[1] * 1e-4 << "\n";
        std::cout << magneticField[2] * 1e-4 << "\n";
    }
}

void phidgets::PhidgetsImu::attachHandler() {
    Phidget::attachHandler();
    // Set device params. This is in attachHandler(), since it has to be
    // repeated on reattachment.
    setDataRate(period_);
}

void phidgets::PhidgetsImu::detachHandler() {
    Phidget::detachHandler();
}

void phidgets::PhidgetsImu::errorHandler(int error) {
    Phidget::errorHandler(error);
}

void phidgets::PhidgetsImu::calibrate() {
    zero();
    // The API call returns directly, so we "enforce" the recommended 2 sec
    // here. See: https://github.com/ros-drivers/phidgets_drivers/issues/40
    std::this_thread::sleep_for(2s);
}

void phidgets::PhidgetsImu::initDevice() {
    openAndWaitForAttachment(-1, 10000);
}

phidgets::PhidgetsImu::PhidgetsImu() : Imu() {
    initDevice();
    calibrate();
    Imu::setCompassCorrectionParameters(cc_mag_field_, cc_offset0_, cc_offset1_, cc_offset2_, cc_gain0_, cc_gain1_, cc_gain2_, cc_T0_, cc_T1_, cc_T2_, cc_T3_, cc_T4_, cc_T5_);
}

