/**
 * @author Marc Eisoldt
 */


#include <stdlib.h>
#include <fstream>
#include <iostream>

#include <hw/buffer/buffer.h>
#include <hw/kernels/vadd_kernel.h>
#include <hw/opencl.h>
#include <hw/fpga_manager.h>
#include <registration/registration.h>
#include <map/local_map.h>
#include <util/pcd/pcd_file.h>
#include <hw/kernels/tsdf_kernel.h>
#include <util/point_hw.h>

#include <tsdf/krnl_tsdf_sw.h>

#include "catch2_config.h"

#include <eigen3/Eigen/Dense>


using fastsense::util::PCDFile;

namespace fastsense::tsdf
{

//static const int DATA_SIZE = 4096;

constexpr unsigned int SCALE = 1000;

constexpr float MAX_OFFSET = 100; // TODO: this is too much

// Test Rotation
constexpr float RY = 5 * (M_PI / 180); //radiants

constexpr float TAU = 1 * SCALE;
constexpr float MAX_WEIGHT = 10 * WEIGHT_RESOLUTION;

constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION; 

static void check_tsdf(const fastsense::buffer::InputOutputBuffer<std::pair<int ,int>>& tsdf_hw, const fastsense::buffer::InputOutputBuffer<std::pair<int, int>>& tsdf_sw)
{
    REQUIRE(tsdf_hw.size() == tsdf_sw.size());

    for(size_t i = 0; i < tsdf_hw.size(); ++i)
    {
        REQUIRE(tsdf_hw[i].first == tsdf_sw[i].first);
    }
}

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Kernel_TSDF", "[kernel][slow]")
{
    std::cout << "Testing 'Kernel_TSDF'" << std::endl;


    fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();

    std::vector<std::vector<Vector3f>> float_points;
    unsigned int num_points;

    fastsense::util::PCDFile file("sim_cloud.pcd");
    file.readPoints(float_points, num_points);

    auto count = 0u;

    ScanPoints_t scan_points(num_points);

    fastsense::buffer::InputBuffer<PointHW> kernel_points(q, num_points);
    //PointHW kernel_points_sw[num_points];

    std::vector<PointHW> kernel_points_sw(num_points);

    for(const auto& ring : float_points)
    {
        for(const auto& point : ring)
        {
            scan_points[count].x() = point.x() * SCALE;
            scan_points[count].y() = point.y() * SCALE;
            scan_points[count].z() = point.z() * SCALE;

            kernel_points[count].x = scan_points[count].x();
            kernel_points[count].y = scan_points[count].y();
            kernel_points[count].z = scan_points[count].z();

            kernel_points_sw[count].x = kernel_points[count].x;
            kernel_points_sw[count].y = kernel_points[count].y;
            kernel_points_sw[count].z = kernel_points[count].z;
            
            ++count;
        }
    }

    std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map.h5", 0.0, 0.0));
    fastsense::map::LocalMap local_map(SIZE_X, SIZE_Y, SIZE_Z, global_map_ptr, q);

    std::shared_ptr<fastsense::map::GlobalMap> global_map_sw_ptr(new fastsense::map::GlobalMap("test_global_map_sw.h5", 0.0, 0.0));
    fastsense::map::LocalMap local_map_sw(SIZE_X, SIZE_Y, SIZE_Z, global_map_sw_ptr, q);

    //calc tsdf values for the points from the pcd and store them in the local map

    fastsense::kernels::TSDFKernel krnl(q);

    krnl.run(local_map, kernel_points, TAU, MAX_WEIGHT);
    krnl.waitComplete();

    fastsense::buffer::InputOutputBuffer<std::pair<int, int>> new_entries(q, local_map_sw.get_size().x() * local_map_sw.get_size().y() * local_map_sw.get_size().z());       

    fastsense::tsdf::krnl_tsdf_sw(kernel_points_sw.data(), 
                                  kernel_points_sw.data(),
                                  num_points,
                                  local_map_sw.getBuffer(),
                                  local_map_sw.getBuffer(),
                                  local_map_sw.get_size().x(), 
                                  local_map_sw.get_size().y(), 
                                  local_map_sw.get_size().z(),
                                  0, 
                                  0, 
                                  0,
                                  local_map_sw.get_offset().x(), 
                                  local_map_sw.get_offset().y(), 
                                  local_map_sw.get_offset().z(),
                                  new_entries,
                                  new_entries,
                                  TAU,
                                  MAX_WEIGHT);
                                  
    check_tsdf(local_map.getBuffer(), local_map_sw.getBuffer());
}

} //namespace fastsense::tsdf