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

using namespace fastsense;
using namespace std::chrono_literals;

TEST_CASE("AccumulatorPos", "[Accumulator]")
{

    msg::ImuStampedBuffer imu_buffer(10);

    auto stamp = util::HighResTime::now();
    auto diff = 250ms;

    msg::Imu imu{msg::LinearAcceleration{0,0,0}, msg::AngularVelocity{0,0,M_PI}, msg::MagneticField{0,0,0}};

    for (const auto& i: {1, 2, 3, 4, 5, 6, 7})
    {
        msg::ImuStamped imu_msg{imu, util::HighResTimePoint(stamp + i * diff)};
        imu_buffer.push(imu_msg);
    }

    registration::ImuAccumulator imu_acc;

    Eigen::Matrix4f acc = imu_acc.acc_transform(imu_buffer, util::HighResTimePoint(stamp + 5 * diff));

    // target rotation in deg
    auto target = 180; 
    // target rotation r in radians
    float r = target * (M_PI / 180); 
    Eigen::Matrix4f rotation_mat;
    rotation_mat <<  cos(r), -sin(r), 0, 0,
                     sin(r),  cos(r), 0, 0,
                     0,       0,      1, 0,
                     0,       0,      0, 1;


    auto rot_in_euler = registration::ImuAccumulator::rot_in_euler(acc);

    REQUIRE(acc(0,0) == Approx(rotation_mat(0,0)).margin(0.001));
    REQUIRE(acc(0,1) == Approx(rotation_mat(0,1)).margin(0.001));
    REQUIRE(acc(0,2) == Approx(rotation_mat(0,2)).margin(0.001));
    REQUIRE(acc(1,0) == Approx(rotation_mat(1,0)).margin(0.001));
    REQUIRE(acc(1,1) == Approx(rotation_mat(1,1)).margin(0.001));
    REQUIRE(acc(1,2) == Approx(rotation_mat(1,2)).margin(0.001));
    REQUIRE(acc(2,0) == Approx(rotation_mat(2,0)).margin(0.001));
    REQUIRE(acc(2,1) == Approx(rotation_mat(2,1)).margin(0.001));
    REQUIRE(acc(2,2) == Approx(rotation_mat(2,2)).margin(0.001));

    REQUIRE(rot_in_euler(2,0) == Approx(-r).margin(0.001));

    REQUIRE(imu_buffer.size() ==  2);
    
    acc = imu_acc.acc_transform(imu_buffer, util::HighResTimePoint(stamp + 7 * diff));

    // 2 imu messages left in buffer -> -90 grad rotation around z axis
    // target rotation in deg
    target = 90; 
    // target rotation r in radians
    r = target * (M_PI / 180); 
    rotation_mat.setIdentity();
    rotation_mat <<  cos(r), -sin(r), 0, 0,
                     sin(r),  cos(r), 0, 0,
                     0,       0,      1, 0,
                     0,       0,      0, 1;


    rot_in_euler.setIdentity();
    rot_in_euler = registration::ImuAccumulator::rot_in_euler(acc);

    REQUIRE(acc(0,0) == Approx(rotation_mat(0,0)).margin(0.000001));
    REQUIRE(acc(0,1) == Approx(rotation_mat(0,1)).margin(0.000001));
    REQUIRE(acc(0,2) == Approx(rotation_mat(0,2)).margin(0.000001));
    REQUIRE(acc(1,0) == Approx(rotation_mat(1,0)).margin(0.000001));
    REQUIRE(acc(1,1) == Approx(rotation_mat(1,1)).margin(0.000001));
    REQUIRE(acc(1,2) == Approx(rotation_mat(1,2)).margin(0.000001));
    REQUIRE(acc(2,0) == Approx(rotation_mat(2,0)).margin(0.000001));
    REQUIRE(acc(2,1) == Approx(rotation_mat(2,1)).margin(0.000001));
    REQUIRE(acc(2,2) == Approx(rotation_mat(2,2)).margin(0.000001));

    REQUIRE(rot_in_euler(2,0) == Approx(r).margin(0.000001));

    REQUIRE(imu_buffer.empty());
}

TEST_CASE("AccumulatorNeg", "[Accumulator]")
{

    msg::ImuStampedBuffer imu_buffer(10);

    auto stamp = util::HighResTime::now();
    auto diff = 250ms;

    msg::Imu imu{msg::LinearAcceleration{0,0,0}, msg::AngularVelocity{0,0,-M_PI}, msg::MagneticField{0,0,0}};

    for (const auto& i: {1, 2, 3, 4, 5, 6, 7})
    {
        msg::ImuStamped imu_msg{imu, util::HighResTimePoint(stamp + i * diff)};
        imu_buffer.push(imu_msg);
    }

    registration::ImuAccumulator imu_acc;

    Eigen::Matrix4f acc = imu_acc.acc_transform(imu_buffer, util::HighResTimePoint(stamp + 5 * diff));

    // target rotation in deg
    auto target = -180; 
    // target rotation r in radians
    float r = target * (M_PI / 180); 
    Eigen::Matrix4f rotation_mat;
    rotation_mat <<  cos(r), -sin(r), 0, 0,
                     sin(r),  cos(r), 0, 0,
                     0,       0,      1, 0,
                     0,       0,      0, 1;


    auto rot_in_euler = registration::ImuAccumulator::rot_in_euler(acc);

    REQUIRE(acc(0,0) == Approx(rotation_mat(0,0)).margin(0.001));
    REQUIRE(acc(0,1) == Approx(rotation_mat(0,1)).margin(0.001));
    REQUIRE(acc(0,2) == Approx(rotation_mat(0,2)).margin(0.001));
    REQUIRE(acc(1,0) == Approx(rotation_mat(1,0)).margin(0.001));
    REQUIRE(acc(1,1) == Approx(rotation_mat(1,1)).margin(0.001));
    REQUIRE(acc(1,2) == Approx(rotation_mat(1,2)).margin(0.001));
    REQUIRE(acc(2,0) == Approx(rotation_mat(2,0)).margin(0.001));
    REQUIRE(acc(2,1) == Approx(rotation_mat(2,1)).margin(0.001));
    REQUIRE(acc(2,2) == Approx(rotation_mat(2,2)).margin(0.001));

    REQUIRE(rot_in_euler(2,0) == Approx(-r).margin(0.001));

    REQUIRE(imu_buffer.size() ==  2);
    
    acc = imu_acc.acc_transform(imu_buffer, util::HighResTimePoint(stamp + 7 * diff));

    // 2 imu messages left in buffer -> -90 grad rotation around z axis

    // target rotation in deg
    target = -90; 
    // target rotation r in radians
    r = target * (M_PI / 180); 
    rotation_mat.setIdentity();
    rotation_mat <<  cos(r), -sin(r), 0, 0,
                     sin(r),  cos(r), 0, 0,
                     0,       0,      1, 0,
                     0,       0,      0, 1;


    rot_in_euler.setIdentity();
    rot_in_euler = registration::ImuAccumulator::rot_in_euler(acc);

    REQUIRE(acc(0,0) == Approx(rotation_mat(0,0)).margin(0.000001));
    REQUIRE(acc(0,1) == Approx(rotation_mat(0,1)).margin(0.000001));
    REQUIRE(acc(0,2) == Approx(rotation_mat(0,2)).margin(0.000001));
    REQUIRE(acc(1,0) == Approx(rotation_mat(1,0)).margin(0.000001));
    REQUIRE(acc(1,1) == Approx(rotation_mat(1,1)).margin(0.000001));
    REQUIRE(acc(1,2) == Approx(rotation_mat(1,2)).margin(0.000001));
    REQUIRE(acc(2,0) == Approx(rotation_mat(2,0)).margin(0.000001));
    REQUIRE(acc(2,1) == Approx(rotation_mat(2,1)).margin(0.000001));
    REQUIRE(acc(2,2) == Approx(rotation_mat(2,2)).margin(0.000001));

    REQUIRE(rot_in_euler(2,0) == Approx(r).margin(0.000001));

    REQUIRE(imu_buffer.empty());
}