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

    /*
     * Write some tsdf values and weights into the local map,
     * that will be written to the file as one chunk (-1_0_0)
     *
     *    y
     *    ^
     *  4 | .   .   .   . | .   .
     *    | Chunk -1_0_0  | Chunk 0_0_0
     *  3 | .   .   .   . | .   .
     *    |               |
     *  2 | .   .  p0  p1 | .   .
     *    |               |
     *  1 | .   .  p2  p3 | .   .
     *    |               |
     *  0 | .   .  p4  p5 | .   .
     *    | --------------+------
     * -1 | .   .   .   . | .   .
     *    | Chunk -1_-1_0 | Chunk 0_-1_0
     * -2 | .   .   .   . | .   .
     *    +-----------------------> x
     *   / -4  -3  -2  -1   0   1
     * z=0
     */
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

    // Test unmanipulated map
    auto pos0 = localMap.get_pos();
    auto size0 = localMap.get_size();
    CHECK(pos0[0] == 0);
    CHECK(pos0[1] == 0);
    CHECK(pos0[2] == 0);
    CHECK(size0[0] == 5);
    CHECK(size0[1] == 5);
    CHECK(size0[2] == 5);
    CHECK(localMap.in_bounds(0, 2, -2));
    CHECK(!localMap.in_bounds(22, 0, 0));
    CHECK(localMap.value(0, 0, 0).value() == 4);
    CHECK(localMap.value(0, 0, 0).weight() == 6);
    CHECK(localMap.value(-1, 2, 0).value() == 1);
    CHECK(localMap.value(-1, 2, 0).weight() == 1);

    // Manipulate the map by applying a kernel that doubles the tsdf values and halfes the weights and shifting
    auto q = FPGAManager::create_command_queue();
    LocalMapTestKernel krnl{q};
    krnl.run(localMap);
    krnl.waitComplete();
    localMap.shift(24, 0, 0);

    // Test manipulated map
    auto pos1 = localMap.get_pos();
    auto size1 = localMap.get_size();
    CHECK(pos1[0] == 24);
    CHECK(pos1[1] == 0);
    CHECK(pos1[2] == 0);
    CHECK(size1[0] == 5);
    CHECK(size1[1] == 5);
    CHECK(size1[2] == 5);
    CHECK(!localMap.in_bounds(0, 2, -2));
    CHECK(localMap.in_bounds(22, 0, 0));
    CHECK(localMap.value(24, 0, 0).value() == 4);
    CHECK(localMap.value(24, 0, 0).weight() == 6);

    // Test persistent storage in HDF5 file
    localMap.write_back();

    HighFive::File f("MapTest.h5", HighFive::File::OpenOrCreate);
    HighFive::Group g = f.getGroup("/map");
    HighFive::DataSet d = g.getDataSet("-1_0_0");
    std::vector<TSDFValue::RawType> chunk;
    d.read(chunk);

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
}
