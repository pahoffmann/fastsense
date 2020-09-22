/**
 * @file registration.cpp
 * @author Patrick Hoffmann
 * @author Malte Hillmann
 * @author Marc Eisoldt
 * @brief Registration Class used to Register a Pointcloud with the current ringbuffer
 * @version 0.1
 * @date 2020-08-24
 *
 * @copyright Copyright (c) 2020
 *
 */


#include "registration.h"

using namespace fastsense::registration;

Registration::~Registration()
{
    // Currently not necessary, because all member variablesare of a memory save type
}


void Registration::transform_point_cloud(ScanPoints_t<Vector3>& in_cloud, const Matrix4x4& transform)
{
    #pragma omp parallel for schedule(static) collapse(2)
    for (auto ring = 0u; ring < in_cloud.size(); ++ring)
    {
        for (auto index = 0u; index < in_cloud[0].size(); ++index)
        {
            Eigen::Vector4f v;
            auto& point = in_cloud[ring][index];

            v << point, 1.0f;
            point = (transform * v).block<3, 1>(0, 0);
        }
    }
}


Registration::Matrix4x4 Registration::xi_to_transform(Matrix6x1 xi)
{
    // Formula 3.9 on Page 40 of "Truncated Signed Distance Fields Applied To Robotics"

    auto angular_velocity = xi.block<3, 1>(0, 0);
    auto theta = angular_velocity.norm();
    auto l = angular_velocity / theta;
    Matrix3x3 L;
    L <<
      0, -l.z(), l.y(),
      l.z(), 0, -l.x(),
      -l.y(), l.x(), 0;

    Matrix4x4 transform = Matrix4x4::Identity();

    auto rotation = transform.block<3, 3>(0, 0);
    rotation += sin(theta) * L + (1 - cos(theta)) * L * L;

    transform.block<3, 1>(0, 3) = xi.block<3, 1>(3, 0);
    return transform;
}

float Registration::filter_value(const std::pair<float, float>& buf_entry)
{
    if (buf_entry.second == 0.0)
    {
        throw - 1;
    }

    return buf_entry.first;
}

float Registration::interpolate(const fastsense::map::LocalMap<std::pair<float, float>>& localmap, const Vector3& point)
{
    int buf_x = (int)std::floor(point.x());
    int buf_y = (int)std::floor(point.y());
    int buf_z = (int)std::floor(point.z());

    return interpolate(localmap, point, buf_x, buf_y, buf_z);
}
float Registration::interpolate(const fastsense::map::LocalMap<std::pair<float, float>>& localmap, const Vector3& point, int buf_x, int buf_y, int buf_z)
{
    // TSDF values must be calculated for a corner not the center of the grid

    float c000;
    float c100;
    float c010;
    float c110;
    float c001;
    float c101;
    float c011;
    float c111;

    try
    {
        c000 = filter_value(localmap.value(buf_x, buf_y, buf_z));
        c100 = filter_value(localmap.value(buf_x + 1, buf_y, buf_z));
        c010 = filter_value(localmap.value(buf_x, buf_y + 1, buf_z));
        c110 = filter_value(localmap.value(buf_x + 1, buf_y + 1, buf_z));
        c001 = filter_value(localmap.value(buf_x, buf_y, buf_z + 1));
        c101 = filter_value(localmap.value(buf_x + 1, buf_y, buf_z + 1));
        c011 = filter_value(localmap.value(buf_x, buf_y + 1, buf_z + 1));
        c111 = filter_value(localmap.value(buf_x + 1, buf_y + 1, buf_z + 1));
    }
    catch (int)
    {
        throw;
    }

    float tx = point.x() - buf_x;
    float ty = point.y() - buf_y;
    float tz = point.z() - buf_z;

    return (1.0 - tx) * (1.0 - ty) * (1.0 - tz) * c000 +
           tx * (1.0 - ty) * (1.0 - tz) * c100 +
           (1.0 - tx) * ty * (1.0 - tz) * c010 +
           tx * ty * (1.0 - tz) * c000 +
           (1.0 - tx) * (1.0 - ty) * tz * c001 +
           tx * (1.0 - ty) * tz * c101 +
           (1.0 - tx) * ty * tz * c011 +
           tx * ty * tz * c111;
}

Registration::Matrix4x4 Registration::register_cloud(const fastsense::map::LocalMap<std::pair<float, float>>& localmap, ScanPoints_t<Vector3>& cloud)
{
    mutex_.lock();
    Matrix4x4 cur_transform = global_transform_; //transform used to register the pcl
    global_transform_.setIdentity();
    mutex_.unlock();

    Matrix4x4 next_transform = cur_transform;

    auto alpha = it_weight_offset_;

    double previous_errors[4] = {0, 0, 0, 0};
    double error = 0.0;
    int count = 0;
    const double epsilon = 0.0001; // TODO: make parameter?
    const size_t width = cloud[0].size();

    bool finished = false;

    Matrix6x6 h = Matrix6x6::Zero();
    Matrix6x1 g = Matrix6x1::Zero();
    Matrix6x1 xi;

    // instead of splitting and joining threads twice per iteration, stay in a
    // multithreaded environment and guard the appropriate places with #pragma omp single
    #pragma omp parallel
    {
        Matrix6x6 local_h;
        Matrix6x1 local_g;
        double local_error;
        int local_count;

        Matrix3x1 gradient;
        Matrix6x1 jacobi;
        Eigen::Vector4f extended;

        for (int i = 0; i < max_iterations_ && !finished; i++)
        {
            local_h = Matrix6x6::Zero();
            local_g = Matrix6x1::Zero();
            local_error = 0.0;
            local_count = 0;

            #pragma omp for schedule(static) nowait
            for (size_t i = 0; i < width; i++)
            {
                for (size_t ring = 0; ring < cloud.size(); ring++)
                {
                    Vector3& point = cloud[ring][i];
                    if (std::isnan(point.x()))
                    {
                        continue;
                    }

                    // apply transform from previous iteration/IMU
                    extended << point, 1.0f;
                    point = (next_transform * extended).block<3, 1>(0, 0);

                    int buf_x = (int)std::floor(point.x());
                    int buf_y = (int)std::floor(point.y());
                    int buf_z = (int)std::floor(point.z());

                    try
                    {
                        const auto& current = localmap.value(buf_x, buf_y, buf_z);
                        if (current.second == 0.0)
                        {
                            continue;
                        }

                        const auto& x_next = localmap.value(buf_x + 1, buf_y, buf_z);
                        const auto& x_last = localmap.value(buf_x - 1, buf_y, buf_z);
                        const auto& y_next = localmap.value(buf_x, buf_y + 1, buf_z);
                        const auto& y_last = localmap.value(buf_x, buf_y - 1, buf_z);
                        const auto& z_next = localmap.value(buf_x, buf_y, buf_z + 1);
                        const auto& z_last = localmap.value(buf_x, buf_y, buf_z - 1);

                        gradient = Matrix3x1::Zero();

                        if (x_next.second != 0.0 && x_last.second != 0.0 && !((x_next.first > 0.0 && x_last.first < 0.0) || (x_next.first < 0.0 && x_last.first > 0.0)))
                        {
                            gradient.x() = (x_next.first - x_last.first) / 2;
                        }
                        if (y_next.second != 0.0 && y_last.second != 0.0 && !((y_next.first > 0.0 && y_last.first < 0.0) || (y_next.first < 0.0 && y_last.first > 0.0)))
                        {
                            gradient.y() = (y_next.first - y_last.first) / 2;
                        }
                        if (z_next.second != 0.0 && z_last.second != 0.0 && !((z_next.first > 0.0 && z_last.first < 0.0) || (z_next.first < 0.0 && z_last.first > 0.0)))
                        {
                            gradient.z() = (z_next.first - z_last.first) / 2;
                        }

                        jacobi << point.cross(gradient), gradient;

                        auto weight = calc_weight(current.first);
                        local_h += weight * jacobi * jacobi.transpose();
                        local_g += weight * jacobi * current.first;
                        local_error += fabs(current.first);
                        local_count++;
                    }
                    catch (std::out_of_range)
                    {
                    }
                }
            }
            // write local results back into shared variables
            #pragma omp critical
            {
                h += local_h;
                g += local_g;
                error += local_error;
                count += local_count;
            }

            // wait for all threads to finish
            #pragma omp barrier
            // only execute on a single thread, all others wait
            #pragma omp single
            {
                // W Matrix
                h += alpha * count * Matrix6x6::Identity();

                xi = -h.inverse() * g; //-h.completeOrthogonalDecomposition().pseudoInverse() * g;

                //convert xi into transform matrix T
                next_transform = xi_to_transform(xi);

                alpha += it_weight_gradient_;

                cur_transform = next_transform * cur_transform; //update transform

                error /= count;
                if (fabs(error - previous_errors[2]) < epsilon && fabs(error - previous_errors[0]) < epsilon)
                {
                    std::cout << "Stopped after " << i << " / " << max_iterations_ << " Iterations" << std::endl;
                    finished = true;
                }
                for (int e = 1; e < 4; e++)
                {
                    previous_errors[e - 1] = previous_errors[e];
                }
                previous_errors[3] = error;

                h = Matrix6x6::Zero();
                g = Matrix6x1::Zero();
                error = 0;
                count = 0;
            }
        }
    }

    // apply final transformation
    transform_point_cloud(cloud, next_transform);

    return cur_transform;
}

/**
 * @brief Gets angluar velocity data from the IMU and stores them in the global_transform object
 *
 * @param imu ROS Message containing the necessary data
 * TODO: auslagern in andere Klasse
 * TODO: determine weather the queue of the pcl callback might be a probl.
 */
void Registration::update_imu_data(const fastsense::msg::ImuMsgStamped& imu)
{
    if(first_imu_msg_ == true){
        imu_time_ = imu.second;
        first_imu_msg_ = false;
        return;
    }

    /*if (imu_time_.toSec() == 0)
    {
        imu_time_ = imu.header.stamp;
        return;
    }*/

    float acc_time = std::chrono::duration_cast<std::chrono::seconds>(imu.second.time - imu_time_.time).count();

    //ros::Duration acc_time = imu.header.stamp - imu_time_;

    Vector3 ang_vel(imu.first.ang.x(), imu.first.ang.y(), imu.first.ang.z());
    //Vector3 ang_vel(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);

    Vector3 orientation = ang_vel * acc_time; //in radiants [rad, rad, rad]
    auto rotation = Eigen::AngleAxisf(orientation.x(), Vector3::UnitX())
                    * Eigen::AngleAxisf(orientation.y(), Vector3::UnitY())
                    * Eigen::AngleAxisf(orientation.z(), Vector3::UnitZ());

    Matrix4x4 local_transform = Matrix4x4::Identity();
    local_transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();

    mutex_.lock();
    global_transform_ *= local_transform; //combine/update transforms
    mutex_.unlock();

    imu_time_ = imu.second;
}
