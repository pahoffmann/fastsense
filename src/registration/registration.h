#include <util/time_stamp.h>
#include <msg/msgs_stamped.h>
#include <map/ring_buffer.h>

#include <cmath>
#include <mutex>
#include <chrono>
#include <utility>
#include <algorithm>


#ifndef REGISTRATION_H_
#define REGISTRATION_H_


/**
 * @brief
 *
 */
class Registration
{

private:
    typedef Eigen::Matrix<float, 6, 1> Matrix6x1; //jacobi, tf
    typedef Eigen::Matrix<float, 6, 3> Matrix6x3; //first part of jacobi
    typedef Eigen::Matrix<float, 3, 1> Matrix3x1; //second part of jacobi
    typedef Eigen::Matrix<float, 6, 6> Matrix6x6; //second part of jacobi
    typedef Eigen::Matrix<float, 4, 4> Matrix4x4; //transform matrix
    typedef Eigen::Matrix<float, 3, 3> Matrix3x3; //rotation matrix

    /*struct Point
    {
        float x;
        float y;
        float z;
        char fill[4];
        float intensity;
        short ring;
        char fill2[10];
    };*/

    int max_iterations_;
    double weighting_constant_;
    double it_weight_offset_;
    double it_weight_gradient_;

    std::mutex mutex_;
    //ros::Publisher marker_pub_;
    Matrix4x4 global_transform_; //used to store the transform since the last registration (right now calculated using the angular velocities by the IMU)
    //ros::Time imu_time_;
    fastsense::util::TimeStamp imu_time_;

    
    /**
     * @brief transforms xi vector 6x1 (angular and linear velocity) to transformation matrix 4x4
     *
     * @param xi vector
     */
    Matrix4x4 xi_to_transform(Matrix6x1 xi);

    static inline float filter_value(const std::pair<float, float>& buf_entry);

    static inline float interpolate(const RingBuffer<std::pair<float, float>>& buffer, const Vector3& point);

    static inline float interpolate(const RingBuffer<std::pair<float, float>>& buffer, const Vector3& point, int buf_x, int buf_y, int buf_z);

public:

    /**
     * @brief Construct a new Registration object, used to register a pointcloud with the current ring buffer
     *
     */
    Registration(unsigned int max_iterations = 500, double weighting_constant = 100.0, double it_weight_offset = 0.0, double it_weight_gradient = 0.01) : 
    max_iterations_(max_iterations),
    weighting_constant_(weighting_constant),
    it_weight_offset_(it_weight_offset),
    it_weight_gradient_(it_weight_gradient) 
    {
        global_transform_.setIdentity();
    }

    //Registration(const ros::NodeHandle& n);

    /**
     * Destructor of the ring buffer.
     * Deletes the array in particular.
     */
    virtual ~Registration();

    //void load_params(const ros::NodeHandle& n);

    float calc_weight(float x) const 
    {
        auto value = fabs(x);

        if(value <= weighting_constant_)
        {
            return 1.0;
        }

        return weighting_constant_ / value;
    }

    /**
     * @brief Registers the given pointcloud with the local ring buffer. Transforms the cloud
     *
     * @param cur_buffer
     * @param cloud
     * @return Matrix4x4
     */
    Matrix4x4 register_cloud(const RingBuffer<std::pair<float, float>>& buffer, ScanPoints_t<Vector3>& cloud);

    /**
     * @brief Updates the IMU data used by the registration method
     *
     * @param imu
     */
    //void update_imu_data(const sensor_msgs::Imu& imu);
    void update_imu_data(const fastsense::msg::ImuMsgStamped imu);


    /**
     * @brief Used to dynamicly reconfigure the parameters used for the registration
     *
     * @param config
     * @param level
     */
    //void dynamic_reconfigure_callback(prototyping::REGConfig& config, uint32_t level);

    /**
     * @brief Transforms a given pointcloud with the transform and stores it inside the out_cloud
     *
     * @param in_cloud
     * @param out_cloud
     * @param transform
     */
    static void transform_point_cloud(ScanPoints_t<Vector3>& in_cloud, const Matrix4x4& transform);
};

#endif
