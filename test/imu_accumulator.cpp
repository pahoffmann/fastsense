#include "catch2_config.h"
#include <msg/imu.h>
#include <msg/angular_velocity.h>
#include <msg/linear_acceleration.h>
#include <msg/magnetic_field.h>
#include <math.h>
#include <registration/imu_accumulator.h>
#include <util/point.h>
#include <iostream>
#include <util/time.h>

using namespace fastsense::msg;

TEST_CASE("Accumulator", "[Accumulator]")
{
    //create imu buffer
    ImuStampedBuffer imu_buffer = ImuStampedBuffer(10);
    
    //fill imu buffer
    Imu imu{LinearAcceleration{0,0,0}, AngularVelocity{0,0,1/5 * M_PI}, MagneticField{0,0,0}};
    ImuStamped imu_msg_1{imu};
    ImuStamped imu_msg_2{imu};
    ImuStamped imu_msg_3{imu};
    ImuStamped imu_msg_4{imu};
    ImuStamped imu_msg_5{imu};

    imu_buffer.push(imu_msg_1);
    imu_buffer.push(imu_msg_2);
    imu_buffer.push(imu_msg_3);
    imu_buffer.push(imu_msg_4);
    imu_buffer.push(imu_msg_5);

    //create imu_accumulator
    fastsense::registration::ImuAccumulator imu_acc;

    Eigen::Matrix4f acc = imu_acc.acc_transform(imu_buffer, fastsense::util::HighResTime::now());

    Eigen::Matrix4f rotation_mat;
    float r = 180 * (M_PI / 180); //radiants
    rotation_mat <<  cos(r), -sin(r),      0, 0,
                 sin(r),  cos(r),      0, 0,
                 0,             0,       1, 0,
                 0,             0,       0, 1;

    std::cout << acc << std::endl;
    std::cout << rotation_mat << std::endl;

    REQUIRE(acc(0,0) == rotation_mat(0,0));
    REQUIRE(acc(0,1) == rotation_mat(0,1));
    REQUIRE(acc(0,2) == rotation_mat(0,2));
    REQUIRE(acc(1,0) == rotation_mat(1,0));
    REQUIRE(acc(1,1) == rotation_mat(1,1));
    REQUIRE(acc(1,2) == rotation_mat(1,2));
    REQUIRE(acc(2,0) == rotation_mat(2,0));
    REQUIRE(acc(2,1) == rotation_mat(2,1));
    REQUIRE(acc(2,2) == rotation_mat(2,2));

}