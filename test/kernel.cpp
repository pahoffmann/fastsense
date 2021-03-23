/**
 * @author Marc Eisoldt
 *
 * Tests the combination of the TSDF Kernel with the registartion kernel
 * with a real point cloud and a simulated translation and rotation
 */


#include <stdlib.h>
#include <hw/kernels/vadd_kernel.h>
#include <registration/registration.h>
#include <util/pcd/pcd_file.h>
#include <hw/kernels/tsdf_kernel.h>

#include "catch2_config.h"

using fastsense::util::PCDFile;

namespace fastsense::registration
{

/// Fixed point scale
constexpr unsigned int SCALE = 1000;

/// Upper bounds for errors
constexpr float MAX_OFFSET = 100;
constexpr float DRIFT_OFFSET = 10;

/// Test Translation
constexpr float TX = 0.3 * SCALE;
constexpr float TY = 0.3 * SCALE;
constexpr float TZ = 0.0 * SCALE;
/// Test Rotation
constexpr float RY = 5 * (M_PI / 180); //radiants

/// TSDF update parameters
constexpr float TAU = 1 * SCALE;
constexpr float MAX_WEIGHT = 10;// * WEIGHT_RESOLUTION;

constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION;

/// Registration parameters
constexpr int MAX_ITERATIONS = 200;

/**
 * @brief Compares two sets of points one-on-one and does some statistics
 *
 * @param points_posttransform Transformed point cloud
 * @param points_pretransform Original point cloud
 * 
 * @return float Average error of the original and the transformed points
 */
float check_computed_transform(const ScanPoints_t& points_posttransform, const ScanPoints_t& points_pretransform, bool print = true)
{
    float minimum = std::numeric_limits<float>::infinity();
    float maximum = -std::numeric_limits<float>::infinity();
    float average = 0;

    float average_x = 0;
    float average_y = 0;
    float average_z = 0;

    std::vector<float> dists(points_pretransform.size());

    for (size_t i = 0; i < points_pretransform.size(); i++)
    {
        Eigen::Vector3i sub = points_pretransform[i] - points_posttransform[i];
        auto norm = sub.cast<float>().norm();

        if (norm < minimum)
        {
            minimum = norm;
        }

        if (norm > maximum)
        {
            maximum = norm;
        }

        average += norm;
        average_x += std::abs(sub.x());
        average_y += std::abs(sub.y());
        average_z += std::abs(sub.z());

        dists[i] = norm;
    }

    std::sort(dists.begin(), dists.end());

    average /= points_pretransform.size();
    average_x /= points_pretransform.size();
    average_y /= points_pretransform.size();
    average_z /= points_pretransform.size();

    if (print)
    {
        std::cout << std::fixed << std::setprecision(2)
                  << "minimum distance: " << minimum << "\n"
                  << "maximum distance: " << maximum << "\n"
                  << "average distance: " << average
                  << ",  (" << (int)average_x << ", " << (int)average_y << ", " << (int)average_z << ")\n"
                  << "median  distance: " << dists[dists.size() / 2 + 1] << std::endl;
    }

    CHECK(average < MAX_OFFSET);

    return average;
}

/**
 * @brief Convert the scan points from the prototype to the hardware system 
 */
static std::shared_ptr<fastsense::buffer::InputBuffer<PointHW>> scan_points_to_input_buffer(const ScanPoints_t& cloud, const fastsense::CommandQueuePtr& q)
{
    auto buffer_ptr = std::make_shared<fastsense::buffer::InputBuffer<PointHW>>(q, cloud.size());
    for (size_t i = 0; i < cloud.size(); i++)
    {
        auto point = cloud[i];
        PointHW tmp(point.x(), point.y(), point.z());
        (*buffer_ptr)[i] = tmp;
    }
    return buffer_ptr;
}

/**
 * @brief Perform a transformation test based on the given set on points, local map and registration lernel
 * 
 * @param points Point set, which should be used for registartion (stored in prototype system)
 * @param transformation_mat Transformation, which should be applied on the point set for testing
 * @param local_map Local map, whoch should be used for registration
 * @param reg Registartion instance, which should be tested
 * @param q Command queue pointer, which should be used to transform the points into the hardware representation
 * 
 * @return float Average error of the original and the transformed points
 */
static float transformation_test(const ScanPoints_t& points, const Eigen::Matrix4f& transformation_mat, fastsense::map::LocalMap& local_map, fastsense::registration::Registration& reg, const fastsense::CommandQueuePtr& q)
{
    // Pretransform the scan points
    ScanPoints_t points_transformed(points);
    reg.transform_point_cloud(points_transformed, transformation_mat);

    // Copy from scan points to inputbuffer
    auto buffer_ptr = scan_points_to_input_buffer(points_transformed, q);
    auto& buffer = *buffer_ptr;
    
    // Register test scan
    Matrix4f result_matrix = Matrix4f::Identity();
    reg.register_cloud(local_map, buffer, buffer.size(), util::HighResTime::now(), result_matrix);

    // Retransform the points with the result transformation and compare them with the original
    reg.transform_point_cloud(points_transformed, result_matrix);
    return check_computed_transform(points_transformed, points);
}

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

/// Test all kernel functionality based on a given prerecorded scan
/// First, a TSDF map is created based on the scan
/// Three cases are tested for the registration
/// 1. Idle 
/// 2. Translation
/// 3. Rotation
/// 4. Rotation drift (achived by applying the registration of the opposite transformations)

TEST_CASE("Kernel", "[kernel][slow]")
{
    std::cout << "Testing 'Kernel'" << std::endl;


    fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();

    // Dummy for the IMU (nut used in this test)
    auto buffer = std::make_shared<msg::ImuStampedBuffer>(0);
    // Initialize the registration
    fastsense::registration::Registration reg(q, buffer, MAX_ITERATIONS);

    // Read the recorded scan

    std::vector<std::vector<Vector3f>> float_points;
    unsigned int num_points;

    fastsense::util::PCDFile file("sim_cloud.pcd");
    file.readPoints(float_points, num_points);

    // Convert the scan for the hardware architecture

    auto count = 0u;

    ScanPoints_t points_original(num_points);

    auto q2 = fastsense::hw::FPGAManager::create_command_queue();
    fastsense::buffer::InputBuffer<PointHW> kernel_points(q2, num_points);

    // From floating point to fixed point
    for (const auto& ring : float_points)
    {
        for (const auto& point : ring)
        {
            points_original[count].x() = point.x() * SCALE;
            points_original[count].y() = point.y() * SCALE;
            points_original[count].z() = point.z() * SCALE;

            kernel_points[count].x = points_original[count].x();
            kernel_points[count].y = points_original[count].y();
            kernel_points[count].z = points_original[count].z();

            ++count;
        }
    }

    // Create the TSDF map
    std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map.h5", 0.0, 0.0));
    fastsense::map::LocalMap local_map(SIZE_Y, SIZE_Y, SIZE_Z, global_map_ptr, q);

    // Initialize temporary testing variables

    Eigen::Matrix4f idle_mat;
    idle_mat     << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

    Eigen::Matrix4f translation_mat;
    translation_mat << 1, 0, 0, TX,
                       0, 1, 0, TY,
                       0, 0, 1, TZ,
                       0, 0, 0,  1;

    Eigen::Matrix4f rotation_mat;
    rotation_mat <<  cos(RY), -sin(RY), 0, 0,
                     sin(RY),  cos(RY), 0, 0,
                           0,        0, 1, 0,
                           0,        0, 0, 1;

    Eigen::Matrix4f rotation_mat2;
    rotation_mat2 <<  cos(-RY), -sin(-RY), 0, 0,
                      sin(-RY),  cos(-RY), 0, 0,
                             0,         0, 1, 0,
                             0,         0, 0, 1;

    // Calculate TSDF values for the points from the pcd and store them in the local map

    auto q3 = fastsense::hw::FPGAManager::create_command_queue();
    fastsense::kernels::TSDFKernel krnl(q3, local_map.getBuffer().size());

    krnl.run(local_map, kernel_points, kernel_points.size(), TAU, MAX_WEIGHT);
    krnl.waitComplete();

    {
        std::cout << "    Section 'Test Registration No Transform'" << std::endl;
        transformation_test(points_original, idle_mat, local_map, reg, q);
    }

    {
        std::cout << "    Section 'Test Registration Translation'" << std::endl;
        transformation_test(points_original, translation_mat, local_map, reg, q);
    }

    {
        std::cout << "    Section 'Registration test Rotation'" << std::endl;
        transformation_test(points_original, rotation_mat, local_map, reg, q);
    }

    {
        std::cout << "    Section 'Registration test Rotation Drift'" << std::endl;
        auto result1 = transformation_test(points_original, rotation_mat, local_map, reg, q);
        auto result2 = transformation_test(points_original, rotation_mat2, local_map, reg, q);

        // Check drift after multiple registration
        CHECK(fabsf(result1 - result2) < DRIFT_OFFSET);
    }
}

} //namespace fastsense::registration
