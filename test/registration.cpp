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

        //test pointcloud transform 
        std::vector<fastsense::msg::Point> cloud(5);
        std::vector<fastsense::msg::Point> result(5);

        //test registration
        fastsense::registration::Registration reg;
        std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map", 0.0, 0.0));
        fastsense::map::LocalMap<std::pair<int, int>> local_map(5, 5, 5, global_map_ptr, q);
        
        //todo: read cloud using marcs stuff

        //todo: calc tsdf and add to local_map
            
        SECTION("Test Transform PCL")
        {
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
            // Initialize temporary testing variables
            global_map_ptr.reset(new fastsense::map::GlobalMap("test_global_map", 0.0, 0.0));
            
            std::vector<fastsense::msg::Point> points_pretransformed;
            std::vector<std::vector<fastsense::msg::Point>> points;
            unsigned int num_points;
            fastsense::util::PCDFile<fastsense::msg::Point> file("");
            file.readPoints(points, num_points);

            for(auto ring = 0u; ring < points.size(); ++ring)
            {
                for(const auto& point : points[ring])
                {
                    fastsense::msg::Point transformed_p;
                    transformed_p.x = point.x * 1000; //m to mm
                    transformed_p.y = point.y * 1000; //m to mm
                    transformed_p.z = point.z * 1000; //m to mm
                    points_pretransformed.push_back(transformed_p);
                }
            }

            //points_transformed now holds the pretransformed (in mm) points from the pcd

            //calc tsdf values for the points from the pcd and store them in the local map
            //update_tsdf(scan_points, Vector3::Zero(), *buffer_ptr_, TsdfCalculation::PROJECTION_INTER, params_.getTau(), max_weight_);

            //translate 20cm in x and y direction
            int tx = 200; 
            int ty = 200;
            int tz = 0;

            Eigen::Matrix4f translation_mat;
            translation_mat << 1, 0, 0, tx,
                               0, 1, 0, ty,
                               0, 0, 1, tz,
                               0, 0, 0, 1;

            reg.transform_point_cloud(points_pretransformed, translation_mat);

            Eigen::Matrix4f transform_mat = reg.register_cloud(local_map, points_pretransformed);

            int max_offset = 10; //10mm maximum difference 

            REQUIRE(std::abs(transform_mat(0,3) - tx) < max_offset); //x translation smaller than offset
            REQUIRE(std::abs(transform_mat(1,3) - tx) < max_offset); //y translation smaller than offset
            REQUIRE(std::abs(transform_mat(2,3) - tx) < max_offset); //z translation smaller than offset

            //todo: scans need to be processed before put in registration?
            //preprocess_scan_global(model, scan_points, params_.getMapResolution(), Mat4::Identity());
            

        }
        SECTION("Registration test Rotation")
        {
            ///todo
        }
    }

} //namespace fastsense:: registration