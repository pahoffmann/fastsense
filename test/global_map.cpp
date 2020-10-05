/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

/*#include <map/local_map.h>
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
    LocalMap localMap{5, 5, 5, gm_ptr, commandQueue};

    // write some tsdf values and weights into one corner of the ring buffer,
    // that will be written to the file as one chunk
    std::pair<int, int> p0(0, 0);
    std::pair<int, int> p1(1, 1);
    std::pair<int, int> p2(2, 1);
    std::pair<int, int> p3(3, 2);
    std::pair<int, int> p4(4, 3);
    std::pair<int, int> p5(5, 5);
    localMap.value(-2, 2, 0) = p0;
    localMap.value(-1, 2, 0) = p1;
    localMap.value(-2, 1, 0) = p2;
    localMap.value(-1, 1, 0) = p3;
    localMap.value(-2, 0, 0) = p4;
    localMap.value(-1, 0, 0) = p5;

    auto& pos = localMap.getPos();
    auto& size = localMap.getSize();

    // test getter
    CHECK(pos[0] == 0);
    CHECK(pos[1] == 0);
    CHECK(pos[2] == 0);
    CHECK(size[0] == 5);
    CHECK(size[1] == 5);
    CHECK(size[2] == 5);
    // test inBounds
    CHECK(localMap.inBounds(0, 2, -2));
    CHECK(!localMap.inBounds(22, 0, 0));
    // test default values
    CHECK(localMap.value(0, 0, 0).first == 0);
    CHECK(localMap.value(0, 0, 0).second == 7);
    // test value access
    CHECK(localMap.value(-1, 2, 0).first == 1);
    CHECK(localMap.value(-1, 2, 0).second == 1);

    auto q = FPGAManager::create_command_queue();
    LocalMapTestKernel krnl{q};
    krnl.run(localMap);
    krnl.waitComplete();

    // shift so that the chunk gets unloaded
    localMap.shift(24, 0, 0);

    auto pos1 = localMap.getPos();
    auto size1 = localMap.getSize();

    // test getter
    CHECK(pos1[0] == 24);
    CHECK(pos1[1] == 0);
    CHECK(pos1[2] == 0);
    CHECK(size1[0] == 5);
    CHECK(size1[1] == 5);
    CHECK(size1[2] == 5);
    // test inBounds
    CHECK(!localMap.inBounds(0, 2, -2));
    CHECK(localMap.inBounds(22, 0, 0));
    // test values
    CHECK(localMap.value(24, 0, 0).first == 0);
    CHECK(localMap.value(24, 0, 0).second == 7);

    // check file for the numbers
    HighFive::File f("MapTest.h5", HighFive::File::OpenOrCreate);
    HighFive::Group g = f.getGroup("/map");
    HighFive::DataSet d = g.getDataSet("-1_0_0"); // FIXME: Chunk is only in the file when it was fully unloaded
    std::vector<int> chunk;
    d.read(chunk);

    // test pose
    gm_ptr->savePose(8, 13, 21, 34, 55, 89);
    gm_ptr->savePose(144, 233, 377, 610, 987, 1597);
    g = f.getGroup("/poses");
    d = g.getDataSet("1");

    std::vector<float> pose;
    d.read(pose);

    // FIXME: Cases relative to map parameters
    CHECK(chunk[(16 * 16 * 14 + 16 * 2) * 2] == 0 * 2);
    CHECK(chunk[(16 * 16 * 15 + 16 * 2) * 2] == 1 * 2);
    CHECK(chunk[(16 * 16 * 14 + 16 * 1) * 2] == 2 * 2);
    CHECK(chunk[(16 * 16 * 15 + 16 * 1) * 2] == 3 * 2);
    CHECK(chunk[(16 * 16 * 14 + 16 * 0) * 2] == 4 * 2);
    CHECK(chunk[(16 * 16 * 15 + 16 * 0) * 2] == 5 * 2);

    // FIXME: TODO: We do not use floats 
    CHECK(chunk[(16 * 16 * 14 + 16 * 2) * 2 + 1] == 0 * 0.5f);
    CHECK(chunk[(16 * 16 * 15 + 16 * 2) * 2 + 1] == 1 * 0.5f);
    CHECK(chunk[(16 * 16 * 14 + 16 * 1) * 2 + 1] == 1 * 0.5f);
    CHECK(chunk[(16 * 16 * 15 + 16 * 1) * 2 + 1] == 2 * 0.5f);
    CHECK(chunk[(16 * 16 * 14 + 16 * 0) * 2 + 1] == 3 * 0.5f);
    CHECK(chunk[(16 * 16 * 15 + 16 * 0) * 2 + 1] == 5 * 0.5f);

    int i = 0;
    CHECK(pose[i++] == 144);
    CHECK(pose[i++] == 233);
    CHECK(pose[i++] == 377);
    CHECK(pose[i++] == 610);
    CHECK(pose[i++] == 987);
    CHECK(pose[i++] == 1597);
}*/
