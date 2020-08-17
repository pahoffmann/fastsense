#include "imu.h"
#include "params.h"
#include "imu_msg.h"
#include <thread>
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace phidgets {

Imu::Imu() : Phidget(), imu_handle_(nullptr), is_calibrated_(false), init_compass_(false)
{
    initApi();
    initDevice();
}

void Imu::setDataRate(int rate)
{
    CPhidgetSpatial_setDataRate(imu_handle_, rate);
}

void Imu::zero()
{
    // zero (calibrate) gyro
    CPhidgetSpatial_zeroGyro(imu_handle_);
}

int Imu::SpatialDataHandler(CPhidgetSpatialHandle /* handle */, void *userptr,
                            CPhidgetSpatial_SpatialEventDataHandle *data,
                            int count)
{
    for (int i = 0; i < count; ++i)
    {
        double ts = data[i]->timestamp.seconds +
                    (data[i]->timestamp.microseconds / 1000.0 / 1000.0);
        ((Imu *)userptr)
        ->dataHandler(data[i]->acceleration, data[i]->angularRate,
                      data[i]->magneticField, ts);
    }
    return 0;
}

void Imu::setCompassCorrectionParameters(double cc_mag_field, double cc_offset0,
        double cc_offset1, double cc_offset2,
        double cc_gain0, double cc_gain1,
        double cc_gain2, double cc_T0,
        double cc_T1, double cc_T2,
        double cc_T3, double cc_T4,
        double cc_T5)
{
    int ret = CPhidgetSpatial_setCompassCorrectionParameters(
                  imu_handle_, cc_mag_field, cc_offset0, cc_offset1, cc_offset2, cc_gain0,
                  cc_gain1, cc_gain2, cc_T0, cc_T1, cc_T2, cc_T3, cc_T4, cc_T5);

    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Compass correction parameter error: " + Phidget::getErrorDescription(ret));
    }
}

using namespace std::chrono_literals;

// TODO detach handler? How to handle connected/disconnectedness

void
phidgets::Imu::dataHandler(const double *acceleration, const double *angularRate, const double *magneticField,
                                   double timestamp) {
    // Even during calibration, the device reports back zeroes, so force driver to wait until
    // after calibration
    if (not is_calibrated_) return;

    std::cout << "=========== ( " << timestamp << ")\n-- acc --\n";
    std::cout << -acceleration[0] * params::G << "\n";
    std::cout << -acceleration[1] * params::G << "\n";
    std::cout << -acceleration[2] * params::G << "\n";

    std::cout << "-- ang --\n";
    std::cout << angularRate[0] * (M_PI / 180.0) << "\n";
    std::cout << angularRate[1] * (M_PI / 180.0) << "\n";
    std::cout << angularRate[2] * (M_PI / 180.0) << "\n";

//    if (magneticField[0] != PUNK_DBL)
//    {
        // device reports data in Gauss, multiply by 1e-4 to convert to Tesla
//        std::cout << "-- mag -- \n";
//        std::cout << magneticField[0] * 1e-4 << "\n";
//        std::cout << magneticField[1] * 1e-4 << "\n";
//        std::cout << magneticField[2] * 1e-4 << "\n";
//    }


}

void phidgets::Imu::attachHandler() {
    Phidget::attachHandler();
    // Set device params. This is in attachHandler(), since it has to be
    // repeated on reattachment.
    setDataRate(params::period_);
}

void phidgets::Imu::detachHandler() {
    Phidget::detachHandler();
}

void phidgets::Imu::errorHandler(int error) {
    Phidget::errorHandler(error);
}

void phidgets::Imu::calibrate() {
    zero();
    // The API call returns directly, so we "enforce" the recommended 2 sec
    // here. See: https://github.com/ros-drivers/phidgets_drivers/issues/40
    std::this_thread::sleep_for(2s);
    is_calibrated_ = true;
}

void phidgets::Imu::initDevice() {
    openAndWaitForAttachment(-1, 10000);

    calibrate();

    if (init_compass_) {
        using namespace phidgets::params;
        Imu::setCompassCorrectionParameters(cc_mag_field_, cc_offset0_, cc_offset1_, cc_offset2_, cc_gain0_, cc_gain1_,
                                            cc_gain2_, cc_T0_, cc_T1_, cc_T2_, cc_T3_, cc_T4_, cc_T5_);
    }
}

void Imu::initApi() {
    // create the handle
    CPhidgetSpatial_create(&imu_handle_);

    // pass handle to base class
    Phidget::init((CPhidgetHandle)imu_handle_);

    // register base class callbacks
    Phidget::registerHandlers();

    // register imu data callback
    CPhidgetSpatial_set_OnSpatialData_Handler(imu_handle_, SpatialDataHandler,
                                              this);
}


}  // namespace phidgets
