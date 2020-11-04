/**
 * @author Marc Eisoldt
 * @author Malte Hillmann
 * @author Marcel Flottmann
 * 
 * Visualize the results of the hardware implementation of the TSDF generation and update 
 * based on a real point cloud via the TSDF bridge
 */
#include "catch2_config.h"

#include <msg/point_cloud.h>
#include <tsdf/update_tsdf.h>
#include <util/pcd/pcd_file.h>
#include <comm/queue_bridge.h>
#include <tsdf/krnl_tsdf_sw.h>
#include <msg/tsdf_bridge_msg.h>
#include <util/config/config_manager.h>

using namespace fastsense;

TEST_CASE("TSDF_Kernel_Vis", "[tsdf_kernel_vis]")
{
    std::cout << "Testing 'TSDF_Kernel_Vis'" << std::endl;   
    SECTION("Visualize TSDF Data")
    {
        std::cout << "    Section 'Visualize TSDF Data'" << std::endl;
        constexpr unsigned int SCALE = 1000;
        constexpr float TAU = 1 * SCALE;
        constexpr float MAX_WEIGHT = 10 * WEIGHT_RESOLUTION;

        constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION + 1;
        constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION + 1;
        constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION + 1; 

        std::vector<std::vector<Vector3f>> float_points;
        unsigned int num_points;

        fastsense::util::PCDFile file("sim_cloud.pcd");
        file.readPoints(float_points, num_points);

        auto count = 0u;

        ScanPoints_t scan_points(num_points);

        auto queue = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::buffer::InputBuffer<PointHW> kernel_points(queue, num_points);

        std::vector<PointHW> kernel_points_sw(num_points);

        for (const auto& ring : float_points)
        {
            for (const auto& point : ring)
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

        std::cout << "num points: " << count << std::endl;

        fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();
        std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr_compare(new fastsense::map::GlobalMap("test_global_map", 0.0, 0.0));
        fastsense::map::LocalMap local_map_compare(SIZE_X, SIZE_Y, SIZE_Z, global_map_ptr_compare, q);

        // Initialize temporary testing variables

        //calc tsdf values for the points from the pcd and store them in the local map

        fastsense::tsdf::update_tsdf(scan_points, Vector3i::Zero(), local_map_compare, TAU, MAX_WEIGHT);

        auto tsdf_buffer = std::make_shared<fastsense::util::ConcurrentRingBuffer<fastsense::msg::TSDFBridgeMessage>>(2);
        fastsense::comm::QueueBridge<fastsense::msg::TSDFBridgeMessage, true> tsdf_bridge{tsdf_buffer, nullptr, 6666};

        tsdf_bridge.start();

        fastsense::CommandQueuePtr q2 = fastsense::hw::FPGAManager::create_command_queue();    
        std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map2", 0.0, 0.0));
        fastsense::map::LocalMap local_map(SIZE_X, SIZE_Y, SIZE_Z, global_map_ptr, q2);

        // auto q3 = fastsense::hw::FPGAManager::create_command_queue();
        // fastsense::kernels::TSDFKernel krnl(q3);

        // krnl.run(local_map, kernel_points, TAU, MAX_WEIGHT);
        // krnl.waitComplete();

        fastsense::buffer::InputOutputBuffer<std::pair<int, int>> new_entries(q, local_map.get_size().x() * local_map.get_size().y() * local_map.get_size().z());       

        for (int i = 0; i < local_map.get_size().x() * local_map.get_size().y() * local_map.get_size().z(); ++i)
        {
            new_entries[i].first = 0;
            new_entries[i].second = 0;
        }

        fastsense::tsdf::krnl_tsdf_sw(kernel_points_sw.data(), 
                                    kernel_points_sw.data(),
                                    num_points,
                                    local_map.getBuffer(),
                                    local_map.getBuffer(),
                                    local_map.get_size().x(), 
                                    local_map.get_size().y(), 
                                    local_map.get_size().z(),
                                    0, 
                                    0, 
                                    0,
                                    local_map.get_offset().x(), 
                                    local_map.get_offset().y(), 
                                    local_map.get_offset().z(),
                                    new_entries,
                                    new_entries,
                                    TAU,
                                    MAX_WEIGHT);

        fastsense::msg::TSDFBridgeMessage tsdf_msg;

        tsdf_msg.tau_ = TAU;
        tsdf_msg.size_[0] = SIZE_X;
        tsdf_msg.size_[1] = SIZE_Y;
        tsdf_msg.size_[2] = SIZE_Z;
        tsdf_msg.pos_[0] = 0;
        tsdf_msg.pos_[1] = 0;
        tsdf_msg.pos_[2] = 0;
        tsdf_msg.offset_[0] = SIZE_X / 2;
        tsdf_msg.offset_[1] = SIZE_Y / 2;
        tsdf_msg.offset_[2] = SIZE_Z / 2;
        tsdf_msg.tsdf_data_.reserve(SIZE_X * SIZE_Y * SIZE_Z);
        std::copy(local_map.getBuffer().cbegin(), local_map.getBuffer().cend(), std::back_inserter(tsdf_msg.tsdf_data_));

        tsdf_buffer->push_nb(tsdf_msg, true);

        tsdf_bridge.stop();
    }
}