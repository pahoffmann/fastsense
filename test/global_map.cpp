/**
 * @file test_global_map.cpp
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include <map/global_map.h>
#include <map/ring_buffer.h>

using namespace fastsense::map;

int main(int argc, char** argv)
{
    std::shared_ptr<GlobalMap> gm_ptr(new GlobalMap("src/prototyping/tmp/test.h5", 0, 7)); 
    auto& gm = *gm_ptr;

    RingBuffer<std::pair<float, float>> rb(5, 5, 5, gm_ptr);

    // write some tsdf values and weights into one corner of the ring buffer,
    // that will be written to the file as one chunk
    std::pair<float, float> p0(0, 0);
    std::pair<float, float> p1(1, 1);
    std::pair<float, float> p2(2, 1);
    std::pair<float, float> p3(3, 2);
    std::pair<float, float> p4(4, 3);
    std::pair<float, float> p5(5, 5);
    rb.value(-2, 2, 0) = p0;
    rb.value(-1, 2, 0) = p1;
    rb.value(-2, 1, 0) = p2;
    rb.value(-1, 1, 0) = p3;
    rb.value(-2, 0, 0) = p4;
    rb.value(-1, 0, 0) = p5;

    // shift so that the chunk gets unloaded
    rb.shift(24, 0, 0);

    // check file for the numbers
    HighFive::File f("src/prototyping/tmp/test.h5", HighFive::File::OpenOrCreate);
    HighFive::Group g = f.getGroup("/map");
    HighFive::DataSet d = g.getDataSet("-1_0_0");
    std::vector<float> chunk;
    d.read(chunk);
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

    // test pose
    gm.savePose(8, 13, 21, 34, 55, 89);
    gm.savePose(144, 233, 377, 610, 987, 1597);
    g = f.getGroup("/poses");
    d = g.getDataSet("1");
    std::vector<float> pose;
    d.read(pose);
    for (int i = 0; i < pose.size(); i++)
    {
        std::cout << pose[i] << " ";
    }
    std::cout << std::endl;
}
