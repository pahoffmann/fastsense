/**
 * @file global_map.cpp
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include <map/local_map.h>
#include <hw/fpga_manager.h>
#include "catch2_config.h"

#include "kernels/local_map_test_kernel.h"

using namespace fastsense::map;
using namespace fastsense::hw;
using namespace fastsense::kernels;

TEST_CASE("Test Map", "[Map]")
{
    std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 7);
    auto commandQueue = FPGAManager::createCommandQueue();
    LocalMap<std::pair<float, float>> localMap{5, 5, 5, gm_ptr, commandQueue};

    // write some tsdf values and weights into one corner of the ring buffer,
    // that will be written to the file as one chunk
    std::pair<float, float> p0(0, 0);
    std::pair<float, float> p1(1, 1);
    std::pair<float, float> p2(2, 1);
    std::pair<float, float> p3(3, 2);
    std::pair<float, float> p4(4, 3);
    std::pair<float, float> p5(5, 5);
    localMap.value(-2, 2, 0) = p0;
    localMap.value(-1, 2, 0) = p1;
    localMap.value(-2, 1, 0) = p2;
    localMap.value(-1, 1, 0) = p3;
    localMap.value(-2, 0, 0) = p4;
    localMap.value(-1, 0, 0) = p5;

    auto& pos = localMap.getPos();
    auto& size = localMap.getSize();

    // test getter
    REQUIRE(pos.x() == 0);
    REQUIRE(pos.y() == 0);
    REQUIRE(pos.z() == 0);
    REQUIRE(size.x() == 5);
    REQUIRE(size.y() == 5);
    REQUIRE(size.z() == 5);
    // test inBounds
    REQUIRE(localMap.inBounds(0, 2, -2));
    REQUIRE(!localMap.inBounds(22, 0, 0));
    // test default values
    REQUIRE(localMap.value(0, 0, 0).first == 0);
    REQUIRE(localMap.value(0, 0, 0).second == 7);
    // test value access
    REQUIRE(localMap.value(-1, 2, 0).first == 1);
    REQUIRE(localMap.value(-1, 2, 0).second == 1);

    auto q = FPGAManager::createCommandQueue();
    LocalMapTestKernel krnl{q};
    krnl.run(localMap);
    krnl.waitComplete();

    // shift so that the chunk gets unloaded
    localMap.shift(24, 0, 0);

    auto pos1 = localMap.getPos();

    // test getter
    REQUIRE(pos1.x() == 24);
    REQUIRE(pos1.y() == 0);
    REQUIRE(pos1.z() == 0);
    REQUIRE(size.x() == 5);
    REQUIRE(size.y() == 5);
    REQUIRE(size.z() == 5);
    // test inBounds
    REQUIRE(!localMap.inBounds(0, 2, -2));
    REQUIRE(localMap.inBounds(22, 0, 0));
    // test values
    
    /*for(int i = -2; i <= 2; ++i)
    {
        for(int j = -2; j <= 2; ++j)
        {
            for(int z = -2; z <= 2; ++z)
            {
                std::cout << i << " " << j << " " << z << ": " << localMap.value(pos1.x() + i, pos1.y() + j, pos1.z() + z).first << " " << localMap.value(pos1.x() + i, pos1.y() + j, pos1.z() + z).second << std::endl;
            }

            std::cout << std::endl;
        }

        std::cout << std::endl;
    }*/
   
    REQUIRE(localMap.value(24, 0, 0).first == 0);
    REQUIRE(localMap.value(24, 0, 0).second == 7);

    // check file for the numbers
    HighFive::File f("MapTest.h5", HighFive::File::OpenOrCreate);
    HighFive::Group g = f.getGroup("/map");
    HighFive::DataSet d = g.getDataSet("-1_0_0");
    std::vector<float> chunk;
    d.read(chunk);

    
    /*std::cout << "tsdf values:" << std::endl;
    for (int y = 15; y >= 0; y--)
    {
        for (int x = 0; x <= 15; x++)
        {
            std::cout << chunk[(16 * 16 * x + 16 * y) * 2] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "weights:" << std::endl;
    for (int y = 15; y >= 0; y--)
    {
        for (int x = 0; x <= 15; x++)
        {
            std::cout << chunk[(16 * 16 * x + 16 * y) * 2 + 1] << " ";
        }
        std::cout << std::endl;
    }*/
    

    // test pose
    gm_ptr->savePose(8, 13, 21, 34, 55, 89);
    gm_ptr->savePose(144, 233, 377, 610, 987, 1597);
    g = f.getGroup("/poses");
    d = g.getDataSet("1");

    std::vector<float> pose;
    d.read(pose);

    /*
    for (size_t i = 0; i < pose.size(); i++)
    {
        std::cout << pose[i] << " ";
    }
    std::cout << std::endl;
    */

    REQUIRE(chunk[(16 * 16 * 14 + 16 * 2) * 2] == 0 * 2);
    REQUIRE(chunk[(16 * 16 * 15 + 16 * 2) * 2] == 1 * 2);
    REQUIRE(chunk[(16 * 16 * 14 + 16 * 1) * 2] == 2 * 2);
    REQUIRE(chunk[(16 * 16 * 15 + 16 * 1) * 2] == 3 * 2);
    REQUIRE(chunk[(16 * 16 * 14 + 16 * 0) * 2] == 4 * 2);
    REQUIRE(chunk[(16 * 16 * 15 + 16 * 0) * 2] == 5 * 2);

    REQUIRE(chunk[(16 * 16 * 14 + 16 * 2) * 2 + 1] == 0 * 0.5f);
    REQUIRE(chunk[(16 * 16 * 15 + 16 * 2) * 2 + 1] == 1 * 0.5f);
    REQUIRE(chunk[(16 * 16 * 14 + 16 * 1) * 2 + 1] == 1 * 0.5f);
    REQUIRE(chunk[(16 * 16 * 15 + 16 * 1) * 2 + 1] == 2 * 0.5f);
    REQUIRE(chunk[(16 * 16 * 14 + 16 * 0) * 2 + 1] == 3 * 0.5f);
    REQUIRE(chunk[(16 * 16 * 15 + 16 * 0) * 2 + 1] == 5 * 0.5f);

    int i = 0;
    REQUIRE(pose[i++] == 144);
    REQUIRE(pose[i++] == 233);
    REQUIRE(pose[i++] == 377);
    REQUIRE(pose[i++] == 610);
    REQUIRE(pose[i++] == 987);
    REQUIRE(pose[i++] == 1597);
}
