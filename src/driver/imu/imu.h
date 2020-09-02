/**
 * @file imu.h
 * @author Julian Gaal
 * @date 2020-08-12
 */

#pragma once

#include <driver/imu/api/phidget.h>
#include <util/msg/msgs_stamped.h>
#include <util/process_thread.h>
#include <data/sensor_sync.h>
#include <util/concurrent_ring_buffer.h>

// TODO singleton
// TODO start function

namespace fastsense::driver
{

/**
 * @brief implements driver phidgets imu 1044, phidgets library version 2.1.9.20190409
 */
class Imu : public Phidget, public ProcessThread
{
public:
    Imu() = delete;

    /**
     * Creates Imu instance
     * @param ringbuffer
     */
    explicit Imu(fastsense::data::ImuStampedBufferPtr ringbuffer);

    virtual ~Imu() = default;

    inline bool isCalibrated() const
    {
        return is_calibrated_;
    }

    void init();

    void start() override;

    void stop() override;

    /**
     * Getter for angular velocity covariance
     * @return angular velocity covariance
     */
    const std::array<double, 9>& getAngularVelocityCovariance() const;

    /**
     * Getter for linear acceleration covariance
     * @return linear acceleration covariance
     */
    const std::array<double, 9>& getLinearAccelerationCovariance() const;

    /**
     * Getter for magnetic field covariance
     * @return magnetic field covariance
     */
    const std::array<double, 9>& getMagneticFieldCovariance() const;

private:
    /// buffer, in which imu readings are saved
    fastsense::data::ImuStampedBufferPtr data_buffer;

    /// whether or not imu is connected
    bool is_connected_;

    /// whether or not imu is calibrated
    bool is_calibrated_;

    /// whether or not to init compass with custom params defined in params.h
    bool init_compass_;

    /// TODO
    int error_number_;

    /// angular velocity covariance
    std::array<double, 9> angular_velocity_covariance_;

    /// linear acceleration covariance
    std::array<double, 9> linear_acceleration_covariance_;

    /// magnetic field covariance
    std::array<double, 9> magnetic_field_covariance_;

    /// handle for phidgets "spatial" unit: imu
    CPhidgetSpatialHandle imu_handle_;

    /**
     * Datahandler for raw data coming from libphidget
     * @param spatial *nused**
     * @param userptr pointer, with which data will be handled furt
     * @param data incoming data
     * @param count how many sensor readings have been made
     */
    static int SpatialDataHandler(CPhidgetSpatialHandle spatial, void* userptr,
                                  CPhidgetSpatial_SpatialEventDataHandle* data,
                                  int count);

//    bool use_imu_time_; // TODO check if necessary! Or only to do with ROS?

    /// calibrate gyro
    void zero();

    /**
     * Set data rate of imu
     * @param rate in ms
     */
    void setDataRate(int rate);

    /// calibrate with zero() and wait, see hee: https://github.com/ros-drivers/phidgets_drivers/issues/40
    void calibrate();

    /**
     * Initialize imu
     */
    void initDevice();

    /**
     * Datahandler, which gets raw data from libphidget
     * @param acceleration linear acceleration
     * @param angularRate angular velocity
     * @param magneticField magnetic field
     * @param timestamp timestamp in sec since initiation
     */
    void dataHandler(const double acceleration[3], const double angularRate[3],
                     const double magneticField[3], double timestamp);

    /// handles attachment of imu
    void attachHandler() override;

    /// handles detachment of imu
    void detachHandler() override;

    /// handles errors from imu
    void errorHandler(int error) override;

    /**
     * Set compass correction parameters for magnetic field, see params.h
     * @param cc_mag_field
     * @param cc_offset0
     * @param cc_offset1
     * @param cc_offset2
     * @param cc_gain0
     * @param cc_gain1
     * @param cc_gain2
     * @param cc_T0
     * @param cc_T1
     * @param cc_T2
     * @param cc_T3
     * @param cc_T4
     * @param cc_T5
     */
    void setCompassCorrectionParameters(double cc_mag_field, double cc_offset0,
                                        double cc_offset1, double cc_offset2,
                                        double cc_gain0, double cc_gain1,
                                        double cc_gain2, double cc_T0,
                                        double cc_T1, double cc_T2, double cc_T3,
                                        double cc_T4, double cc_T5);

    /// initiate libphigdet api
    void initApi();

    /// initialize covariance matrices
    void initCovariance();
};

} // namespace fastsense::driver;