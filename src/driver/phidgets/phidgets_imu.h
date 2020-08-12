#ifndef PHIDGETS_IMU_IMU_ROS_I_H
#define PHIDGETS_IMU_IMU_ROS_I_H

#include "api/imu.h"
#include <memory>
#include <mutex>

namespace phidgets {

const float G = 9.80665;
const float MAX_TIMEDIFF_SECONDS = 0.1;

class PhidgetsImu final : public Imu
{
  public:
    explicit PhidgetsImu();

  private:
    // diagnostics
    bool is_connected_;
    int error_number_;


    // params
    // taken directly from https://github.com/ros-drivers/phidgets_drivers/blob/melodic/phidgets_imu/launch/imu.launch
    static constexpr int serial_number_ = -1;

    static constexpr int period_ = 4;  // rate in ms

    double angular_velocity_stdev_;
    double linear_acceleration_stdev_;
    double magnetic_field_stdev_;
//    bool use_imu_time_; // TODO check if necessary! Or only to do with ROS?

    // compass correction params (see
    // http://www.phidgets.com/docs/1044_User_Guide)
    static constexpr double cc_mag_field_ = 0.52859;
    static constexpr double cc_offset0_ = 0.03921;
    static constexpr double cc_offset1_ = 0.19441;
    static constexpr double cc_offset2_ = -0.03493;
    static constexpr double cc_gain0_ = 1.81704;
    static constexpr double cc_gain1_ = 1.81028;
    static constexpr double cc_gain2_ = 2.04819;
    static constexpr double cc_T0_ = 0.00142;
    static constexpr double cc_T1_ = -0.03591;
    static constexpr double cc_T2_ = 0.00160;
    static constexpr double cc_T3_ = -0.05038;
    static constexpr double cc_T4_ = -0.03942;
    static constexpr double cc_T5_ = -0.05673;


    void calibrate();
    void initDevice();
    void dataHandler(const double acceleration[3], const double angularRate[3],
                     const double magneticField[3], double timestamp) override;
    void attachHandler() override;
    void detachHandler() override;
    void errorHandler(int error) override;
};

}  // namespace phidgets

#endif  // PHIDGETS_IMU_IMU_ROS_I_H
