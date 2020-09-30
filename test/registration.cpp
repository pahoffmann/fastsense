/**
 * @author Patrick Hoffmann
 */


#include <stdlib.h>
#include <fstream>
#include <iostream>

#include <hw/buffer/buffer.h>
#include <hw/kernels/vadd_kernel.h>
#include <hw/types.h>
#include <hw/fpga_manager.h>
#include <registration/registration.h>
#include <msg/point.h>
#include <map/local_map.h>
#include <util/pcd/PCDFile.h>
#include <tsdf/update_tsdf.h>

#include "catch2_config.h"

#include <eigen3/Eigen/Dense>


using fastsense::msg::Point;
using fastsense::util::PCDFile;

namespace fastsense::registration
{

//static const int DATA_SIZE = 4096;

constexpr unsigned int SCALE = 1000;

constexpr float MAX_OFFSET = 0.1 * SCALE;

// Test Translation
constexpr float TX = 0.5 * SCALE;
constexpr float TY = 0.5 * SCALE;
constexpr float TZ = 0.0 * SCALE;
// Test Rotation
constexpr float RY = 15 * (M_PI / 180); //radiants

constexpr float TAU = 1 * SCALE;
constexpr float MAX_WEIGHT = 10 * WEIGHT_RESOLUTION;

constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION; 

/**
 * @brief Compares two matrices (transform and registered transform) and checks wether they match considering a certain amount of drift
 * 
 * @param a 
 * @param b 
 * @param trans_offset 
 * @param rot_offset 
 */
void compare_mats(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b, float trans_offset, float rot_offset)
{
    //std::cout << a << std::endl << std::endl;
    //std::cout << b << std::endl;

    //TODO:: Force marc to do this.
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            REQUIRE(std::abs(a(i, j) - b(i, j)) < rot_offset); //x translation smaller than offset
        }
        
        REQUIRE(std::abs(a(i, 3) - b(i, 3)) < trans_offset);
    }
}

void check_computed_transform(const Eigen::Matrix4f &transform, const ScanPoints_t& points_posttransform, ScanPoints_t& points_pretransform, float max_dist = MAX_OFFSET)
{
    //fastsense::registration::Registration::transform_point_cloud(points_pretransform, transform);
    for (size_t i = 0; i < points_pretransform.size(); i++)
    {
        Eigen::Vector3i sub = points_pretransform[i] - points_posttransform[i];
        auto norm = sub.norm();

        //std::cout << sub.x() << " " << sub.y() << " " << sub.z() << std::endl;

        REQUIRE(norm < max_dist);
    }
    
}

void the_ultimate_compare_mats_function_v3(const Eigen::Matrix4f &a, const Eigen::Matrix4f& b, std::vector<fastsense::msg::Point> points)
{
    REQUIRE(a == b);
}

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Registration", "[registration][slow]")
{
    // const char* xclbinFilename = "FastSense.xclbin";

    // fs::hw::FPGAManager::load_xclbin(xclbinFilename);



    // // These commands will allocate memory on the Device. The cl::Buffer objects can
    // // be used to reference the memory locations on the device.
    // fs::buffer::InputBuffer<int> buffer_a{q, DATA_SIZE};
    // fs::buffer::InputBuffer<int> buffer_b{q, DATA_SIZE};
    // fs::buffer::OutputBuffer<int> buffer_result{q, DATA_SIZE};

    // //setting input data
    // for (int i = 0 ; i < DATA_SIZE; i++)
    // {
    //     buffer_a[i] = 10;
    //     buffer_b[i] = 20;
    // }

    // fs::kernels::VaddKernel krnl_vadd{q};

    // krnl_vadd.run(buffer_a, buffer_b, buffer_result, DATA_SIZE);
    // krnl_vadd.waitComplete();

    // //Verify the result
    // for (int i = 0; i < DATA_SIZE; i++)
    // {
    //     int host_result = buffer_a[i] + buffer_b[i];
    //     REQUIRE(buffer_result[i] == host_result);
    // }


    // //create a registration object:
    // Registration registration = new Registration();

    // //use registration object to calculate a specific transformation, for example: create a pointcloud, transform it and calculate the transformation using the registration
    // //might be a basic cloud with random data - or to make it random, but repeatable: use seeds.


    // create pcl from velodyne sample, create local map, transform pcl and see what the reconstruction can do.

    fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();

    //test registration
    fastsense::registration::Registration reg(10);

    std::vector<std::vector<Vector3f>> float_points;
    unsigned int num_points;

    fastsense::util::PCDFile file("sim_cloud.pcd");
    file.readPoints(float_points, num_points);

    auto count = 0u;

    ScanPoints_t scan_points(num_points);
    ScanPoints_t scan_points_2(num_points);

    std::cout << __LINE__ << std::endl;

    for(const auto& ring : float_points)
    {
        for(const auto& point : ring)
        {
            scan_points[count].x() = point.x() * SCALE;
            scan_points[count].y() = point.y() * SCALE;
            scan_points[count].z() = point.z() * SCALE;

            scan_points_2[count].x() = point.x() * SCALE;
            scan_points_2[count].y() = point.y() * SCALE;
            scan_points_2[count].z() = point.z() * SCALE;
            
            ++count;
        }
    }

    std::cout << __LINE__ << std::endl;


    ScanPoints_t points_pretransformed_trans(scan_points);
    ScanPoints_t points_pretransformed_rot(scan_points);

    std::cout << __LINE__ << std::endl;

    std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map", 0.0, 0.0));
    fastsense::map::LocalMap local_map(SIZE_Y, SIZE_Y, SIZE_Z, global_map_ptr, q);

    std::cout << __LINE__ << std::endl;

    // Initialize temporary testing variables

    Eigen::Matrix4f translation_mat;
    translation_mat << 1, 0, 0, TX,
                       0, 1, 0, TY,
                       0, 0, 1, TZ,
                       0, 0, 0,  1;

    Eigen::Matrix4f rotation_mat;
    rotation_mat <<  cos(RY), 0, sin(RY), 0,
                           0, 1,       0, 0,
                    -sin(RY), 0, cos(RY), 0,
                           0, 0,       0, 1;

    //calc tsdf values for the points from the pcd and store them in the local map

    std::cout << __LINE__ << std::endl;

    fastsense::tsdf::update_tsdf(scan_points, Vector3i::Zero(), local_map, TAU, MAX_WEIGHT);

    std::cout << __LINE__ << std::endl;

    SECTION("Test Transform PCL")
    {
        std::cout << __LINE__ << std::endl;

        //test pointcloud transform
        ScanPoints_t cloud(5);
        ScanPoints_t result(5);

        float tx = 2.0f * SCALE;
        float ty = 2.0f * SCALE;
        float tz = 2.0f * SCALE;
        Eigen::Matrix4f translation_mat;
        translation_mat << 1, 0, 0, tx,
                           0, 1, 0, ty,
                           0, 0, 1, tz,
                           0, 0, 0, 1;

        cloud[0] = Vector3i{1, 1, 1} * SCALE;
        cloud[1] = Vector3i{0, 0, 0} * SCALE;
        cloud[2] = Vector3i{5, 3, 1} * SCALE;
        cloud[3] = Vector3i{540, 244, 124} * SCALE;
        cloud[4] = Vector3i{1, 0, 0} * SCALE;

        reg.transform_point_cloud(cloud, translation_mat);

        result[0] = Vector3i{3, 3, 3} * SCALE;
        result[1] = Vector3i{2, 2, 2} * SCALE;
        result[2] = Vector3i{7, 5, 3} * SCALE;
        result[3] = Vector3i{542, 246, 126} * SCALE;
        result[4] = Vector3i{3, 2, 2} * SCALE;

        for (auto i = 0; i < cloud.size(); i++)
        {
            REQUIRE(cloud[i].x() == result[i].x());
            REQUIRE(cloud[i].y() == result[i].y());
            REQUIRE(cloud[i].z() == result[i].z());
        }
        //reg.register_cloud(local_map, cloud);

        float rx = 90 * (M_PI / 180); //radiants
        Eigen::Matrix4f rotation_mat;
        rotation_mat <<  cos(rx), 0, sin(rx), 0,
                               0, 1,       0, 0,
                        -sin(rx), 0, cos(rx), 0,
                               0, 0,       0, 1;

        cloud[0] = Vector3i{1000, 1000, 1000} * SCALE;

        reg.transform_point_cloud(cloud, rotation_mat);

        result[0] = Vector3i{1000, 1000, -1000} * SCALE;

        REQUIRE(cloud[0].x() == result[0].x());
        REQUIRE(cloud[0].y() == result[0].y());
        REQUIRE(cloud[0].z() == result[0].z());

        std::cout << __LINE__ << std::endl;
    }

    SECTION("Test Registration Translation")
    {
        std::cout << __LINE__ << std::endl;

        reg.transform_point_cloud(points_pretransformed_trans, translation_mat);
        Eigen::Matrix4f transform_mat = reg.register_cloud(local_map, points_pretransformed_trans);
        compare_mats(translation_mat, transform_mat, MAX_OFFSET, MAX_OFFSET);

        std::cout << translation_mat << std::endl << std::endl;
        std::cout << transform_mat << std::endl;

        //check_computed_transform(transform_mat, points_pretransformed_trans, scan_points);

        std::cout << __LINE__ << std::endl;
    }

    SECTION("Registration test Rotation")
    {
        std::cout << __LINE__ << std::endl;

        reg.transform_point_cloud(points_pretransformed_rot, rotation_mat);
        Eigen::Matrix4f transform_mat = reg.register_cloud(local_map, points_pretransformed_rot);
        //compare_mats(rotation_mat, transform_mat, MAX_OFFSET, MAX_OFFSET);
        check_computed_transform(transform_mat, points_pretransformed_rot, scan_points_2);

        
        std::cout << __LINE__ << std::endl;
    }
}

} //namespace fastsense::registration  
