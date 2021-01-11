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

void Registration::register_cloud(fastsense::map::LocalMap& localmap,
                                  fastsense::buffer::InputBuffer<PointHW>& cloud,
                                  const util::HighResTimePoint& cloud_timestamp,
                                  Matrix4f& pose)
{
    Matrix4f imu_rotation = imu_accumulator_.acc_transform(cloud_timestamp);

    Matrix4f move_back = Matrix4f::Identity();
    Matrix4f move_there = Matrix4f::Identity();
    move_back.block<3, 1>(0, 3) = -pose.block<3, 1>(0, 3);
    move_there.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3);

    // imu_rotation rotates around the current position => Translate pos to origin, rotate, translate back
    Matrix4f estimate = move_there * imu_rotation * move_back * pose;
    pose = estimate;

    krnl.synchronized_run(localmap, cloud, max_iterations_, it_weight_gradient_, epsilon_, pose);

    // apply final transformation
    transform_point_cloud(cloud, pose);
}