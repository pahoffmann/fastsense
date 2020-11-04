/**
 * @file test/preprocessing.cpp
 * @author Pascal Buschermöhle
 */

#include "catch2_config.h"
#include <preprocessing/preprocessing.h>
#include <util/point.h>
#include <msg/point_cloud.h>

using namespace fastsense::preprocessing;
using fastsense::ScanPoint;

TEST_CASE("Preprocessing", "[Preprocessing]"){
    REQUIRE(1 == 1);

    Preprocessing preprocessor;
    fastsense::msg::PointCloud::Ptr cloud = std::make_shared<fastsense::msg::PointCloud>();
    fastsense::msg::PointCloudStamped cloud_stamped;
    cloud_stamped.first = cloud;

    std::vector<ScanPoint> points(24);
    //RING 1
    points[0] = {0, 0, 10}; //d = 10
    points[2] = {20, 20, 30}; //d = 41.231056
    points[4] = {0, 0, 30}; //d = 30
    points[6] = {30, 30, 10}; //d = 43.588989
    points[8] = {30, 30, 30}; //d = 51.961524
    points[10] = {10, 10, 10}; //d = 17.320508
    points[12] = {10, 10, 30}; //d = 33.166248
    points[14] = {20, 20, 10}; //d = 30
    points[16] = {40, 40, 10}; //d = 57.445626
    points[18] = {50, 50, 10}; //d = 71.414284
    points[20] = {40, 40, 30}; //d = 64.031242
    points[22] = {50, 50, 30}; //d = 76.811457

    //RING 2
    points[1] = {0, 0, 20}; //d = 20
    points[3] = {20, 20, 20}; //d = 34.641016
    points[5] = {20, 20, 40}; //d = 48.989795
    points[7] = {30, 30, 20}; //d = 46.904158
    points[9] = {30, 30, 40}; //d = 58.309519
    points[11] = {40, 40, 20}; //d = 60
    points[13] = {40, 40, 40}; //d = 69.282032
    points[15] = {0, 0, 40}; //d = 40
    points[17] = {10, 10, 20}; //d = 24.494897
    points[19] = {10, 10, 40}; //d = 42.426407
    points[21] = {50, 50, 40}; //d = 81.240384
    points[23] = {50, 50, 20}; //d = 73.484692
    


    //cloud->rings_ = 2;
    //cloud->points_ = points;
    cloud_stamped.first->points_ = points;
    cloud_stamped.first->rings_ = 2;

    SECTION("Test median filter"){

        preprocessor.median_filter(cloud_stamped, 5);

        std::vector<ScanPoint> result(24);
        result[0] = {20, 20, 30}; //d = 41.231056
        result[2] = {20, 20, 30}; //d = 41.231056
        result[4] = {20, 20, 30}; //d = 41.231056
        result[6] = {20, 20, 30}; //d = 43.588989
        result[8] = {10, 10, 30}; //d = 33.166248
        result[10] = {10, 10, 30}; //d = 33.166248
        result[12] = {10, 10, 30}; //d = 33.166248
        result[14] = {10, 10, 30}; //d = 33.166248
        result[16] = {40, 40, 10}; //d = 57.445626
        result[18] = {40, 40, 30}; //d = 64.031242
        result[20] = {40, 40, 30}; //d = 64.031242
        result[22] = {40, 40, 30}; //d = 64.031242

        result[1] = {20, 20, 40}; //d = 48.989795
        result[3] = {30, 30, 20}; //d = 46.904158
        result[5] = {30, 30, 20}; //d = 46.904158
        result[7] = {20, 20, 40}; //d = 48.989795
        result[9] = {30, 30, 40}; //d = 58.309519
        result[11] = {30, 30, 40}; //d = 58.309519
        result[13] = {30, 30, 40}; //d = 58.309519
        result[15] = {10, 10, 40}; //d = 42.426407
        result[17] = {10, 10, 40}; //d = 42.426407
        result[19] = {10, 10, 40}; //d = 42.426407
        result[21] = {10, 10, 40}; //d = 42.426407
        result[23] = {10, 10, 40}; //d = 42.426407

        for(uint32_t i = 0; i < result.size(); i++)
        {
            REQUIRE(cloud_stamped.first->points_[i] == result[i]);
        }
    }




    SECTION("Test reduction filter"){

    }

}