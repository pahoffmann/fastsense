/**
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
    auto commandQueue = FPGAManager::create_command_queue();
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

    int pos[3];
    localMap.getPos(pos);
    int size[3];
    localMap.getSize(size);

    // test getter
    REQUIRE(pos[0] == 0);
    REQUIRE(pos[1] == 0);
    REQUIRE(pos[2] == 0);
    REQUIRE(size[0] == 5);
    REQUIRE(size[1] == 5);
    REQUIRE(size[2] == 5);
    // test inBounds
    REQUIRE(localMap.inBounds(0, 2, -2));
    REQUIRE(!localMap.inBounds(22, 0, 0));
    // test default values
    REQUIRE(localMap.value(0, 0, 0).first == 0);
    REQUIRE(localMap.value(0, 0, 0).second == 7);
    // test value access
    REQUIRE(localMap.value(-1, 2, 0).first == 1);
    REQUIRE(localMap.value(-1, 2, 0).second == 1);

    auto q = FPGAManager::create_command_queue();
    LocalMapTestKernel krnl{q};
    krnl.run(localMap);
    krnl.waitComplete();

    // shift so that the chunk gets unloaded
    localMap.shift(24, 0, 0);

    localMap.getPos(pos);
    // test getter
    REQUIRE(pos[0] == 24);
    REQUIRE(pos[1] == 0);
    REQUIRE(pos[2] == 0);
    REQUIRE(size[0] == 5);
    REQUIRE(size[1] == 5);
    REQUIRE(size[2] == 5);
    // test inBounds
    REQUIRE(!localMap.inBounds(0, 2, -2));
    REQUIRE(localMap.inBounds(22, 0, 0));
    // test values
    REQUIRE(localMap.value(24, 0, 0).first == 0);
    REQUIRE(localMap.value(24, 0, 0).second == 7);

    // check file for the numbers
    HighFive::File f("MapTest.h5", HighFive::File::OpenOrCreate);
    HighFive::Group g = f.getGroup("/map");
    HighFive::DataSet d = g.getDataSet("-1_0_0");
    std::vector<float> chunk;
    d.read(chunk);

    /*
    std::cout << "tsdf values:" << std::endl;
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
    }
    */

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
