/**
 * @author Patrick Hoffmann
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <util/point.h>
#include <registration/registration.h>
#include <util/runtime_evaluator.h>

using namespace fastsense;
using namespace fastsense::registration;
using namespace fastsense::util;

Registration::Registration(fastsense::CommandQueuePtr q, msg::ImuStampedBuffer::Ptr& buffer, unsigned int max_iterations, float it_weight_gradient, float epsilon)
    :
    max_iterations_(max_iterations),
    it_weight_gradient_(it_weight_gradient),
    epsilon_(epsilon),
    imu_accumulator_(buffer),
    krnl{q}
{
}

void Registration::transform_point_cloud(fastsense::ScanPoints_t& in_cloud, const Matrix4f& mat)
{
    #pragma omp parallel for schedule(static)
    for (auto index = 0u; index < in_cloud.size(); ++index)
    {
        auto& point = in_cloud[index];
        Vector3f tmp = (mat.block<3, 3>(0, 0) * point.cast<float>() + mat.block<3, 1>(0, 3));
        tmp[0] < 0 ? tmp[0] -= 0.5 : tmp[0] += 0.5;
        tmp[1] < 0 ? tmp[1] -= 0.5 : tmp[1] += 0.5;
        tmp[2] < 0 ? tmp[2] -= 0.5 : tmp[2] += 0.5;

        point = tmp.cast<int>();
    }
}

void Registration::transform_point_cloud(fastsense::buffer::InputBuffer<PointHW>& in_cloud, const Matrix4f& mat)
{
    // TODO: test if this is faster on HW
    #pragma omp parallel for schedule(static)
    for (auto index = 0u; index < in_cloud.size(); ++index)
    {
        auto& point = in_cloud[index];
        Vector3f eigen_point(point.x, point.y, point.z);
        Vector3f tmp = (mat.block<3, 3>(0, 0) * eigen_point + mat.block<3, 1>(0, 3));
        tmp[0] < 0 ? tmp[0] -= 0.5 : tmp[0] += 0.5;
        tmp[1] < 0 ? tmp[1] -= 0.5 : tmp[1] += 0.5;
        tmp[2] < 0 ? tmp[2] -= 0.5 : tmp[2] += 0.5;

        point.x = static_cast<int>(tmp.x());
        point.y = static_cast<int>(tmp.y());
        point.z = static_cast<int>(tmp.z());
    }
}


Matrix4f Registration::xi_to_transform(Vector6f xi)
{
    // Formula 3.9 on Page 40 of "Truncated Signed Distance Fields Applied To Robotics"

    // Rotation around an Axis.
    // Direction of axis (l) = Direction of angular_velocity
    // Angle of Rotation (theta) = Length of angular_velocity
    auto angular_velocity = xi.block<3, 1>(0, 0);
    auto theta = angular_velocity.norm();
    auto l = angular_velocity / theta;
    Eigen::Matrix3f L;
    L <<
      0, -l.z(), l.y(),
      l.z(), 0, -l.x(),
      -l.y(), l.x(), 0;

    Matrix4f transform = Matrix4f::Identity();

    auto rotation = transform.block<3, 3>(0, 0);
    rotation += sin(theta) * L + (1 - cos(theta)) * L * L;

    transform.block<3, 1>(0, 3) = xi.block<3, 1>(3, 0);
    return transform;
}

Matrix4f Registration::register_cloud(fastsense::map::LocalMap& localmap, fastsense::buffer::InputBuffer<PointHW>& cloud, const util::HighResTimePoint& cloud_timestamp)
{
    auto& eval = RuntimeEvaluator::get_instance();
    eval.start("imu_acc");
    std::cout << "Imu Buffer size: " << imu_accumulator_.buffer_->size() << "\n";
    Matrix4f total_transform = imu_accumulator_.acc_transform(cloud_timestamp); //transform used to register the pcl
    std::cout << "TOT\n " << total_transform << "\n";
    eval.stop("imu_acc");


    eval.start("reg");
    krnl.synchronized_run(localmap, cloud, max_iterations_, it_weight_gradient_, epsilon_, total_transform);
    // apply final transformation
    transform_point_cloud(cloud, total_transform);
    eval.stop("reg");

    return total_transform;
}