/**
 * @author Patrick Hoffmann
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include "registration.h"

using namespace fastsense::registration;

Vector3i transform(const Vector3i& input, const Matrix4i& mat)
{
    return (mat.block<3, 3>(0, 0) * input + mat.block<3, 1>(0, 3)) / MATRIX_RESOLUTION;
}

//todo: check whether data exchange has a negative consequences regarding the runtime
//TODO: test functionality
//TODO: extract to hw. -> no need to copy points to hw
void Registration::transform_point_cloud(ScanPoints_t& in_cloud, const Matrix4f& mat)
{
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

Matrix4f Registration::register_cloud(fastsense::map::LocalMap& localmap, fastsense::buffer::InputBuffer<Point> cloud, fastsense::CommandQueuePtr q)
{
    //std::cout << __LINE__ << std::endl;
    
    mutex_.lock();
    Matrix4f total_transform = imu_accumulator_; //transform used to register the pcl
    imu_accumulator_.setIdentity();
    mutex_.unlock();

    Matrix4f next_transform = total_transform;

    float alpha = 0;

    float previous_errors[4] = {0, 0, 0, 0};
    int error = 0.0;
    int count = 0;
    constexpr float epsilon = 0.0001; // TODO: make parameter?

    bool finished = false;

    Matrix6i h = Matrix6i::Zero();
    Vector6i g = Vector6i::Zero();

    // instead of splitting and joining threads twice per iteration, stay in a
    // multithreaded environment and guard the appropriate places with #pragma omp single
    //#pragma omp parallel
    {
        Matrix6i local_h;
        Vector6i local_g;
        int local_error;
        int local_count;

        Vector3i gradient;
        Vector6i jacobi;

        for (int i = 0; i < max_iterations_ && !finished; i++)
        {
            //std::cout << __LINE__ << std::endl;

            local_h = Matrix6i::Zero();
            local_g = Vector6i::Zero();
            local_error = 0;
            local_count = 0;

            //std::cout << next_transform << std::endl << std::endl;

            //next_transform = (total_transform * MATRIX_RESOLUTION).cast<int>();
           
            //TODO: total transform, used in hw or local transform used on entire pcl - you decide.
            //transform_point_cloud(cloud, next_transform);

            //STOP SW IMPLEMENTATION
            //BEGIN HW IMPLEMENTATION

            //kernel - run
            fastsense::kernels::RegistrationKernel krnl{q};

            //fastsense::buffer::InputBuffer<fastsense::msg::Point> buffer_scan{q, cloud.size()};

            //Output size: local_h matrix (6x6) + local_g matrix (6x1) + local_error (int)  + local_count (int)
            // 36 + 6 + 1 + 1 = 44
            //fastsense::buffer::OutputBuffer<int> buffer_output{q, 44};

            //Write scan points into buffer
            // for(int i = 0; i < cloud.size(); i++){
            //     buffer_scan[i] = cloud[i];
            // }

            //std::cout << __LINE__ << std::endl;

            krnl.synchronized_run(localmap, cloud, local_h, local_g, local_error, local_count, next_transform); 

            //std::cout << __LINE__ << std::endl;

            //krnl.waitComplete();

            //RESUME SOFTWARE IMPLEMENTATION
            // write local results back into shared variables

            h += local_h;
            g += local_g;
            error += local_error;
            count += local_count;

            //std::cout << __LINE__ << std::endl;


            // wait for all threads to finish //here should be the exit point of the hw communication, after which the data is being used.
            //#pragma omp barrier
            // only execute on a single thread, all others wait - use the data coming from hw to calculate diff things.
            //#pragma omp single
            {
                Matrix6f hf = h.cast<float>();
                Vector6f gf = g.cast<float>();

                // W Matrix
                hf += alpha * count * Matrix6f::Identity();

                Vector6f xi = -hf.inverse() * gf;
                // alternative: Vector6f xi = hf.completeOrthogonalDecomposition().solve(-gf);

                //convert xi into transform matrix T
                next_transform = xi_to_transform(xi);

                //std::cout << "Next transform: " <<  next_transform << std::endl;

                total_transform = next_transform * total_transform; //update transform

                alpha += it_weight_gradient_;

                float err = (float)error / count;

                if (fabs(err - previous_errors[2]) < epsilon && fabs(err - previous_errors[0]) < epsilon)
                {
                    //std::cout << "Stopped after " << i << " / " << max_iterations_ << " Iterations" << std::endl;
                    finished = true;
                }
                for (int e = 1; e < 4; e++)
                {
                    previous_errors[e - 1] = previous_errors[e];
                }
                previous_errors[3] = err;

                h = Matrix6i::Zero();
                g = Vector6i::Zero();
                error = 0;
                count = 0;
                            
                //std::cout << __LINE__ << std::endl;
            }
        }
    }

    // apply final transformation
    transform_point_cloud(cloud, next_transform);

    return total_transform;
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
    if (first_imu_msg_ == true)
    {
        imu_time_ = imu.second;
        first_imu_msg_ = false;
        return;
    }

    float acc_time = std::chrono::duration_cast<std::chrono::milliseconds>(imu.second.time - imu_time_.time).count() / 1000.0f;

    Vector3f ang_vel(imu.first.ang.x(), imu.first.ang.y(), imu.first.ang.z());

    Vector3f orientation = ang_vel * acc_time; //in radiants [rad, rad, rad]
    auto rotation =   Eigen::AngleAxisf(orientation.x(), Vector3f::UnitX())
                    * Eigen::AngleAxisf(orientation.y(), Vector3f::UnitY())
                    * Eigen::AngleAxisf(orientation.z(), Vector3f::UnitZ());

    Matrix4f local_transform = Matrix4f::Identity();
    local_transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();

    mutex_.lock();
    imu_accumulator_ = local_transform * imu_accumulator_; //combine/update transforms
    mutex_.unlock();

    imu_time_ = imu.second;
}
