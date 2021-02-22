#pragma once

/**
 * @file preprocessing.h
 * @author Pascal Buschermöhle
 */

#include <util/point_hw.h>
#include <msg/point_cloud.h>
#include <hw/buffer/buffer.h>
#include <util/point.h>
#include <stdlib.h>
#include <algorithm>


namespace fastsense::preprocessing
{

/**
 * @brief This class provides functions to process raw lidar data. 
 *        Namely it provides a reduction filter, a median filter and a function to convert PointCloud points to HWPoints. 
 */
class Preprocessing
{
public:
    /**
     * @brief This function converts PointCloud Points to Hardware Points and writes them into an input buffer
     * 
     * @param cloud point cloud message that contains data points from the lidar
     * @param scan_points input buffer in which the, to hwpoint converted, lidar scan points will be written 
     */
    void preprocess_scan(const fastsense::msg::PointCloudPtrStamped& cloud, fastsense::buffer::InputBuffer<PointHW>& scan_points);

    /**
     * @brief This function reduces the amount of PointCloud points. It achieves this by merging together points which belong to the same voxel (given by MAP_RESOLUTION) 
     *        and replacing them by one single average point. So every voxel contains at most one point after the reduction.
     *        NOTE: This function will destroy the ring structure of the point cloud
     * 
     * @param cloud point cloud message that contains data points from the lidar
     */
    void reduction_filter_average(fastsense::msg::PointCloudPtrStamped& cloud);

    void reduction_filter_voxel_center(fastsense::msg::PointCloudPtrStamped& cloud);

    void reduction_filter_random_point(fastsense::msg::PointCloudPtrStamped& cloud);

    /**
     * @brief This function applies a median filter to the input point cloud. The median filter will be applied ring-wise. So every point in the point cloud
     *        will be replaced by the median point (based of the euclidian distance to the origin) of its neighbours. 
     * 
     * @param cloud point cloud message that contains data points from the lidar
     * @param window_size determines the size of the neighbourhood that will be looked at to find the median point 
     *        NOTE: window_size has to be odd
     */
    void median_filter(fastsense::msg::PointCloudPtrStamped& cloud, uint8_t window_size);

private:

        /**
     * @brief This function applies a median filter to the input point cloud. The median filter will be applied ring-wise. So every point in the point cloud
     *        will be replaced by the median point (based of the euclidian distance to the origin) of its neighbours. 
     * 
     * @param cloud point cloud message that contains data points from the lidar
     * @param window_size determines the size of the neighbourhood that will be looked at to find the median point 
     *        NOTE: window_size has to be odd
     */
    uint8_t median_from_array(std::vector<ScanPoint*> medians);
};

}