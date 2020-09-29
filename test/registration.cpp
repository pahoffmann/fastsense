/**
 * @file registration.cpp
 * @author Patrick
 * @date 2020-08-09
 * Test file used to test the registration implementation
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

namespace fastsense::registration{

static const int DATA_SIZE = 4096;

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Test Registration", "")
{
    // const char* xclbinFilename = "FastSense.xclbin";

    // fs::hw::FPGAManager::loadXCLBIN(xclbinFilename);

    

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
        
    fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::createCommandQueue();

    //test registration
    fastsense::registration::Registration reg(1);

    ScanPoints_t<Vector3> points;
    unsigned int num_points;
    
    fastsense::util::PCDFile<Vector3> file("sim_cloud.pcd");
    file.readPoints(points, num_points);

    std::vector<fastsense::msg::Point> points_pretransformed_trans;

    for(auto ring = 0u; ring < points.size(); ++ring)
    {
        for(const auto& point : points[ring])
        {
            fastsense::msg::Point transformed_p;
            transformed_p.x = point.x() * 10; //m to mm
            transformed_p.y = point.y() * 10; //m to mm
            transformed_p.z = point.z() * 10; //m to mm
            points_pretransformed_trans.push_back(transformed_p);
        }
    }

    std::vector<fastsense::msg::Point> points_pretransformed_rot(points_pretransformed_trans);

    ScanPoints_t<Eigen::Vector3f> scan_points(points.size()); 
    //points_transformed now holds the pretransformed (in mm) points from the pcd
    for(auto ring = 0u; ring < points.size(); ++ring)
    {
        for(const auto& point : points[ring])
        {
            Eigen::Vector3f transformed_p;
            transformed_p.x() = point.x() * 10; //m to mm
            transformed_p.y() = point.y() * 10; //m to mm
            transformed_p.z() = point.z() * 10; //m to mm
            scan_points[ring].push_back(transformed_p);
        }
    }

    std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map", 0.0, 0.0));
    fastsense::map::LocalMap<std::pair<float, float>> local_map(20 * 10, 20 * 10, 5 * 10, global_map_ptr, q);

    // Initialize temporary testing variables

    float tau = 10;
    float max_weight = 10;

    //calc tsdf values for the points from the pcd and store them in the local map

    fastsense::tsdf::update_tsdf(scan_points, Vector3::Zero(), local_map, fastsense::tsdf::TsdfCalculation::PROJECTION_INTER, tau, max_weight);

        SECTION("Test Transform PCL")
        {
            //test pointcloud transform 
            std::vector<fastsense::msg::Point> cloud(5);
            std::vector<fastsense::msg::Point> result(5);

            float tx = 2.0f; 
            float ty = 2.0f;
            float tz = 2.0f;
            Eigen::Matrix4f translation_mat;
            translation_mat << 1, 0, 0, tx,
                               0, 1, 0, ty,
                               0, 0, 1, tz,
                               0, 0, 0, 1;
            
            cloud[0] = Point{1, 1, 1};
            cloud[1] = Point{0, 0, 0};
            cloud[2] = Point{5, 3, 1};
            cloud[3] = Point{540, 244, 124};
            cloud[4] = Point{1, 0, 0};

            reg.transform_point_cloud(cloud, translation_mat);

            result[0] = Point{3, 3, 3};
            result[1] = Point{2, 2, 2};
            result[2] = Point{7, 5, 3};
            result[3] = Point{542, 246, 126};
            result[4] = Point{3, 2, 2};

            for(auto i = 0; i < cloud.size(); i++){
                REQUIRE(cloud[i].x == result[i].x);
                REQUIRE(cloud[i].y == result[i].y);
                REQUIRE(cloud[i].z == result[i].z);
            }
            //reg.register_cloud(local_map, cloud);

            float rx = 90 * (M_PI/180); //radiants
            Eigen::Matrix4f translation_mat_y;
            translation_mat_y << cos(rx),     0, sin(rx), 0,
                            0,           1, 0,       0,
                            -1* sin(rx), 0, cos(rx), 0,
                            0,           0, 0,       1;

            cloud[0] = Point{1000, 1000, 1000};

            reg.transform_point_cloud(cloud, translation_mat_y);

            result[0] = Point{1000, 1000, -1000};

            REQUIRE(cloud[0].x == result[0].x);
            REQUIRE(cloud[0].y == result[0].y);
            REQUIRE(cloud[0].z == result[0].z);
        }

        SECTION("Test Registration Translation")
        {
            //translate 50cm in x and y direction
            int tx = 5; 
            int ty = 5;
            int tz = 0;

            Eigen::Matrix4f translation_mat;
            translation_mat << 1, 0, 0, tx,
                               0, 1, 0, ty,
                               0, 0, 1, tz,
                               0, 0, 0, 1;

            reg.transform_point_cloud(points_pretransformed_trans, translation_mat);

            Eigen::Matrix4f transform_mat = reg.register_cloud(local_map, points_pretransformed_trans);

            int max_offset = 1; // 10cm maximum difference 

            for (size_t i = 0; i < 4; i++)
            {
                for (size_t j = 0; i < 4; i++)
                {
                    REQUIRE(std::abs(translation_mat(i, j) - transform_mat(i, j)) < max_offset); //x translation smaller than offset    
                }
                
            }
        }
        
        SECTION("Registration test Rotation")
        {
            float rx = 10 * (M_PI/180); //radiants
            Eigen::Matrix4f rotation_mat_y;
            rotation_mat_y << cos(rx),     0, sin(rx), 0,
                            0,           1, 0,       0,
                            -1* sin(rx), 0, cos(rx), 0,
                            0,           0, 0,       1;


            reg.transform_point_cloud(points_pretransformed_rot, rotation_mat_y);

            Eigen::Matrix4f transform_mat = reg.register_cloud(local_map, points_pretransformed_rot);

            int max_offset = 1; //10mm maximum difference 

            for (size_t i = 0; i < 4; i++)
            {
                for (size_t j = 0; i < 4; i++)
                {
                    REQUIRE(std::abs(rotation_mat_y(i, j) - transform_mat(i, j)) < max_offset); //x translation smaller than offset    
                }
                
            }
        }
    }

} //namespace fastsense:: registration