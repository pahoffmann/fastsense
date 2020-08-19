#ifndef PHIDGETS_API_IMU_H
#define PHIDGETS_API_IMU_H

#include "api/phidget.h"
#include "msg/imu_msg.h"
#include "../../util/ring_buffer.h"

namespace phidgets {

class Imu : public Phidget
{
public:
    Imu() = delete;

    explicit Imu(ring_buffer<ImuMsg>& ringbuffer);

    virtual ~Imu() = default;


    const std::array<double, 9> & getAngularVelocityCovariance() const;
    const std::array<double, 9> & getLinearAccelerationCovariance() const;
    const std::array<double, 9> & getMagneticFieldCovariance() const;

private:
    ring_buffer<ImuMsg>& data_buffer;
    // diagnostics
    bool is_connected_;
    bool is_calibrated_;
    bool init_compass_;
    int error_number_;

    std::array<double, 9> angular_velocity_covariance_;
    std::array<double, 9> linear_acceleration_covariance_;
    std::array<double, 9> magnetic_field_covariance_;

    CPhidgetSpatialHandle imu_handle_;

    static int SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr,
                                  CPhidgetSpatial_SpatialEventDataHandle *data,
                                  int count);

//    bool use_imu_time_; // TODO check if necessary! Or only to do with ROS?
    void zero();
    void setDataRate(int rate);
    void calibrate();
    void initDevice();
    void dataHandler(const double acceleration[3], const double angularRate[3],
                     const double magneticField[3], double timestamp);
    void attachHandler() override;
    void detachHandler() override;
    void errorHandler(int error) override;
    void setCompassCorrectionParameters(double cc_mag_field, double cc_offset0,
                                        double cc_offset1, double cc_offset2,
                                        double cc_gain0, double cc_gain1,
                                        double cc_gain2, double cc_T0,
                                        double cc_T1, double cc_T2, double cc_T3,
                                        double cc_T4, double cc_T5);

    void initApi();
    void initCovariance();
};

}  // namespace phidgets

#endif  // PHIDGETS_API_IMU_H
