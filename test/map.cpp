/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include "catch2_config.h"
#include "kernels/local_map_test_kernel.h"

using namespace fastsense::map;
using namespace fastsense::hw;
using namespace fastsense::kernels;

TEST_CASE("Map", "[Map]")
{
    std::cout << "Testing 'Map'" << std::endl;
    std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 4, 6);
    auto commandQueue = FPGAManager::create_command_queue();
    LocalMap localMap{5, 5, 5, gm_ptr, commandQueue};

    // write some tsdf values and weights into one corner of the ring buffer,
    // that will be written to the file as one chunk
    TSDFValue p0(0, 0);
    TSDFValue p1(1, 1);
    TSDFValue p2(2, 1);
    TSDFValue p3(3, 2);
    TSDFValue p4(4, 3);
    TSDFValue p5(5, 5);
    localMap.value(-2, 2, 0) = p0;
    localMap.value(-1, 2, 0) = p1;
    localMap.value(-2, 1, 0) = p2;
    localMap.value(-1, 1, 0) = p3;
    localMap.value(-2, 0, 0) = p4;
    localMap.value(-1, 0, 0) = p5;

    auto& pos = localMap.get_pos();
    auto& size = localMap.get_size();

    // test getter
    CHECK(pos[0] == 0);
    CHECK(pos[1] == 0);
    CHECK(pos[2] == 0);
    CHECK(size[0] == 5);
    CHECK(size[1] == 5);
    CHECK(size[2] == 5);
    // test in_bounds
    CHECK(localMap.in_bounds(0, 2, -2));
    CHECK(!localMap.in_bounds(22, 0, 0));
    // test default values
    CHECK(localMap.value(0, 0, 0).value() == 4);
    CHECK(localMap.value(0, 0, 0).weight() == 6);
    // test value access
    CHECK(localMap.value(-1, 2, 0).value() == 1);
    CHECK(localMap.value(-1, 2, 0).weight() == 1);

    auto q = FPGAManager::create_command_queue();
    LocalMapTestKernel krnl{q};
    krnl.run(localMap);
    krnl.waitComplete();

    // shift so that the chunk gets unloaded
    // Each shift can only cover an area of one Map size
    localMap.shift(Eigen::Vector3i( 5, 0, 0));
    localMap.shift(Eigen::Vector3i(10, 0, 0));
    localMap.shift(Eigen::Vector3i(15, 0, 0));
    localMap.shift(Eigen::Vector3i(20, 0, 0));
    localMap.shift(Eigen::Vector3i(24, 0, 0));

    auto pos1 = localMap.get_pos();
    auto size1 = localMap.get_size();

    // test getter
    CHECK(pos1[0] == 24);
    CHECK(pos1[1] == 0);
    CHECK(pos1[2] == 0);
    CHECK(size1[0] == 5);
    CHECK(size1[1] == 5);
    CHECK(size1[2] == 5);
    // test in_bounds
    CHECK(!localMap.in_bounds(0, 2, -2));
    CHECK(localMap.in_bounds(22, 0, 0));
    // test values
    CHECK(localMap.value(24, 0, 0).value() == 4);
    CHECK(localMap.value(24, 0, 0).weight() == 6);

    localMap.write_back();

    // check file for the numbers
    HighFive::File f("MapTest.h5", HighFive::File::OpenOrCreate);
    HighFive::Group g = f.getGroup("/map");
    HighFive::DataSet d = g.getDataSet("-1_0_0");
    std::vector<TSDFValue::RawType> chunk;
    d.read(chunk);

    // test pose
    gm_ptr->save_pose(8, 13, 21, 34, 55, 89, 1);
    gm_ptr->save_pose(144, 233, 377, 610, 987, 1597, 1);
    g = f.getGroup("/poses");
    d = g.getDataSet("1");

    std::vector<float> pose;
    d.read(pose);

    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 2) + GlobalMap::CHUNK_SIZE * 2)]).value() == 0 * 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 1) + GlobalMap::CHUNK_SIZE * 2)]).value() == 1 * 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 2) + GlobalMap::CHUNK_SIZE * 1)]).value() == 2 * 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 1) + GlobalMap::CHUNK_SIZE * 1)]).value() == 3 * 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 2) + GlobalMap::CHUNK_SIZE * 0)]).value() == 4 * 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 1) + GlobalMap::CHUNK_SIZE * 0)]).value() == 5 * 2);

    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 2) + GlobalMap::CHUNK_SIZE * 2)]).weight() == 0 / 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 1) + GlobalMap::CHUNK_SIZE * 2)]).weight() == 1 / 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 2) + GlobalMap::CHUNK_SIZE * 1)]).weight() == 1 / 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 1) + GlobalMap::CHUNK_SIZE * 1)]).weight() == 2 / 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 2) + GlobalMap::CHUNK_SIZE * 0)]).weight() == 3 / 2);
    CHECK(TSDFValue(chunk[(GlobalMap::CHUNK_SIZE * GlobalMap::CHUNK_SIZE * (GlobalMap::CHUNK_SIZE - 1) + GlobalMap::CHUNK_SIZE * 0)]).weight() == 5 / 2);

    int i = 0;
    CHECK(pose[i++] == 144);
    CHECK(pose[i++] == 233);
    CHECK(pose[i++] == 377);
    CHECK(pose[i++] == 610);
    CHECK(pose[i++] == 987);
    CHECK(pose[i++] == 1597);
}
