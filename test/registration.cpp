/**
 * @author Patrick Hoffmann
 * @author Marc Eisoldt
 * @author Pascal Buschermöhle
 * @author Malte Hillmann
 */


#include <stdlib.h>

#include <hw/kernels/vadd_kernel.h>
#include <registration/registration.h>
#include <tsdf/update_tsdf.h>
#include <util/pcd/pcd_file.h>

#include "catch2_config.h"


using fastsense::util::PCDFile;

namespace fastsense::registration
{

//static const int DATA_SIZE = 4096;

constexpr unsigned int SCALE = 1000;

constexpr float MAX_OFFSET = 100; // TODO: this is too much

// Test Translation
constexpr float TX = 0.3 * SCALE;
constexpr float TY = 0.3 * SCALE;
constexpr float TZ = 0.0 * SCALE;
// Test Rotation
constexpr float RY = 10 * (M_PI / 180); //radiants

constexpr float TAU = 1 * SCALE;
constexpr float MAX_WEIGHT = 10;// * WEIGHT_RESOLUTION;

constexpr int MAX_ITERATIONS = 200;

constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION;

constexpr int ACCURACY = 5;

static void check_computed_transform(const ScanPoints_t& points_posttransform, ScanPoints_t& points_pretransform)
{
    int minimum = std::numeric_limits<int>::infinity();
    int maximum = -std::numeric_limits<int>::infinity();
    int average = 0;

    int average_x = 0;
    int average_y = 0;
    int average_z = 0;

    std::vector<int> dists(points_pretransform.size());

    for (size_t i = 0; i < points_pretransform.size(); i++)
    {
        Eigen::Vector3i sub = points_pretransform[i] - points_posttransform[i];
        auto norm = sub.norm();

        if (norm < minimum)
        {
            minimum = norm;
        }

        if (norm > maximum)
        {
            maximum = norm;
        }

        average += norm;
        average_x += std::abs(sub.x());
        average_y += std::abs(sub.y());
        average_z += std::abs(sub.z());

        dists[i] = norm;

        //std::cout << norm << std::endl;
        //REQUIRE(norm < MAX_OFFSET);
    }

    std::sort(dists.begin(), dists.end());

    std::cout << "minimum distance: " << minimum << std::endl;
    std::cout << "maximum distance: " << maximum << std::endl;
    std::cout << "average distance: " << average / points_pretransform.size() << std::endl;
    std::cout << "average distance x: " << average_x / points_pretransform.size() << std::endl;
    std::cout << "average distance y: " << average_y / points_pretransform.size() << std::endl;
    std::cout << "average distance z: " << average_z / points_pretransform.size() << std::endl;
    std::cout << "median distance: " << dists[dists.size() / 2 + 1] << std::endl;

    REQUIRE((average / points_pretransform.size()) < MAX_OFFSET);
}

static std::shared_ptr<fastsense::buffer::InputBuffer<PointHW>> scan_points_to_input_buffer(ScanPoints_t& cloud, const fastsense::CommandQueuePtr q)
{
    auto buffer_ptr = std::make_shared<fastsense::buffer::InputBuffer<PointHW>>(q, cloud.size());
    for (size_t i = 0; i < cloud.size(); i++)
    {
        auto point = cloud[i];
        PointHW tmp(point.x(), point.y(), point.z());
        (*buffer_ptr)[i] = tmp;
    }
    return buffer_ptr;
}

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Registration", "[registration][slow]")
{
    std::cout << "Testing 'Registration'" << std::endl;


    fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();

    auto buffer = std::make_shared<msg::ImuStampedBuffer>(0);
    //test registration
    fastsense::registration::Registration reg(q, buffer, MAX_ITERATIONS);

    std::vector<std::vector<Vector3f>> float_points;
    unsigned int num_points;

    fastsense::util::PCDFile file("sim_cloud.pcd");
    file.readPoints(float_points, num_points);

    auto count = 0u;

    ScanPoints_t scan_points(num_points);

    for (const auto& ring : float_points)
    {
        for (const auto& point : ring)
        {
            scan_points[count].x() = point.x() * SCALE;
            scan_points[count].y() = point.y() * SCALE;
            scan_points[count].z() = point.z() * SCALE;

            ++count;
        }
    }

    ScanPoints_t scan_points_2(scan_points);
    ScanPoints_t points_pretransformed_trans(scan_points);
    ScanPoints_t points_pretransformed_rot(scan_points);

    std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map.h5", 0.0, 0.0));

    fastsense::map::LocalMap local_map(SIZE_Y, SIZE_Y, SIZE_Z, global_map_ptr, q);

    // Initialize temporary testing variables

    Eigen::Matrix4f translation_mat;
    translation_mat << 1, 0, 0, TX,
                    0, 1, 0, TY,
                    0, 0, 1, TZ,
                    0, 0, 0,  1;

    Eigen::Matrix4f rotation_mat;
    rotation_mat <<  cos(RY), -sin(RY),      0, 0,
                 sin(RY),  cos(RY),      0, 0,
                 0,             0,       1, 0,
                 0,             0,       0, 1;

    //calc tsdf values for the points from the pcd and store them in the local map

    fastsense::tsdf::update_tsdf(scan_points, Vector3i::Zero(), local_map, TAU, MAX_WEIGHT);

    SECTION("Test Transform PCL")
    {
        std::cout << "    Section 'Test Transform PCL'" << std::endl;
        //test pointcloud transform
        ScanPoints_t cloud(5);
        ScanPoints_t result(5);

        float tx = 2.0f * SCALE;
        float ty = 2.0f * SCALE;
        float tz = 2.0f * SCALE;

        Eigen::Matrix4f translation_mat;
        translation_mat << 1, 0, 0, tx,
                        0, 1, 0, ty,
                        0, 0, 1, tz,
                        0, 0, 0, 1;

        cloud[0] = Vector3i{1, 1, 1} * SCALE;
        cloud[1] = Vector3i{0, 0, 0} * SCALE;
        cloud[2] = Vector3i{5, 3, 1} * SCALE;
        cloud[3] = Vector3i{540, 244, 124} * SCALE;
        cloud[4] = Vector3i{1, 0, 0} * SCALE;

        reg.transform_point_cloud(cloud, translation_mat);

        result[0] = Vector3i{3, 3, 3} * SCALE;
        result[1] = Vector3i{2, 2, 2} * SCALE;
        result[2] = Vector3i{7, 5, 3} * SCALE;
        result[3] = Vector3i{542, 246, 126} * SCALE;
        result[4] = Vector3i{3, 2, 2} * SCALE;

        for (size_t i = 0u; i < cloud.size(); i++)
        {
            CHECK(cloud[i].x() == result[i].x());
            CHECK(cloud[i].y() == result[i].y());
            CHECK(cloud[i].z() == result[i].z());
        }

        float rx = 90 * (M_PI / 180); //radiants
        Eigen::Matrix4f rotation_mat;
        rotation_mat <<  cos(rx), 0, sin(rx), 0,
                     0, 1,       0, 0,
                     -sin(rx), 0, cos(rx), 0,
                     0, 0,       0, 1;

        cloud[0] = Vector3i{1000, 1000, 1000} * SCALE;

        reg.transform_point_cloud(cloud, rotation_mat);

        result[0] = Vector3i{1000, 1000, -1000} * SCALE;

        CHECK(cloud[0].x() == result[0].x());
        CHECK(cloud[0].y() == result[0].y());
        CHECK(cloud[0].z() == result[0].z());
    }

    SECTION("Test Registration No Transform")
    {
        std::cout << "    Section 'Test Registration No Transform'" << std::endl;

        //copy from scanpoints to  inputbuffer
        auto buffer_ptr = scan_points_to_input_buffer(points_pretransformed_trans, q);
        auto& buffer = *buffer_ptr;
        Matrix4f result_matrix = Matrix4f::Identity();
        reg.register_cloud(local_map, buffer, util::HighResTime::now(), result_matrix);

        reg.transform_point_cloud(points_pretransformed_trans, result_matrix);
        check_computed_transform(points_pretransformed_trans, scan_points);

    }

    SECTION("Test Registration Translation")
    {
        std::cout << "    Section 'Test Registration Translation'" << std::endl;

        reg.transform_point_cloud(points_pretransformed_trans, translation_mat);

        //copy from scanpoints to  inputbuffer
        auto buffer_ptr = scan_points_to_input_buffer(points_pretransformed_trans, q);
        auto& buffer = *buffer_ptr;
        Matrix4f result_matrix = Matrix4f::Identity();
        reg.register_cloud(local_map, buffer, util::HighResTime::now(), result_matrix);

        reg.transform_point_cloud(points_pretransformed_trans, result_matrix);
        check_computed_transform(points_pretransformed_trans, scan_points);

    }

    SECTION("Registration test Rotation")
    {
        std::cout << "    Section 'Registration test Rotation'" << std::endl;
        reg.transform_point_cloud(points_pretransformed_rot, rotation_mat);
        auto buffer_ptr = scan_points_to_input_buffer(points_pretransformed_rot, q);
        auto& buffer = *buffer_ptr;
        Matrix4f result_matrix = Matrix4f::Identity();
        reg.register_cloud(local_map, buffer, util::HighResTime::now(), result_matrix);

        reg.transform_point_cloud(points_pretransformed_rot, result_matrix);
        check_computed_transform(points_pretransformed_rot, scan_points_2);
    }
}

} //namespace fastsense::registration
