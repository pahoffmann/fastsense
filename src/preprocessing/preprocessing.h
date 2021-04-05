#pragma once

/**
 * @file preprocessing.h
 * @author Pascal Buschermöhle
 * @author Malte Hillmann
 */

#include <util/point_hw.h>
#include <msg/point_cloud.h>
#include <hw/buffer/buffer.h>
#include <util/point.h>

#include <stdlib.h>
#include <algorithm>

#include <util/process_thread.h>
#include <comm/queue_bridge.h>

namespace fastsense::preprocessing
{

using PointCloudBuffer = util::ConcurrentRingBuffer<fastsense::msg::PointCloudPtrStamped>;

/**
 * @brief This class provides functions to process raw lidar data.
 *        Namely it provides a reduction filter, a median filter and a function to convert PointCloud points to HWPoints.
 */
class Preprocessing : public comm::QueueBridge<msg::PointCloudPtrStamped, true>
{
public:
    Preprocessing(const std::shared_ptr<PointCloudBuffer>& in_buffer,
                  const std::shared_ptr<PointCloudBuffer>& out_buffer,
                  const std::shared_ptr<PointCloudBuffer>& send_buffer,
                  bool send_original,
                  bool send_preprocessed,
                  float scale = 1.0);

    void thread_run() override;

    /**
     * @brief Reduces the PointCloud with Voxel reduction
     *
     * This function reduces the amount of PointCloud points. It achieves this by merging together points which belong to the same voxel (given by MAP_RESOLUTION)
     * and replacing them by one single average point. So every voxel contains at most one point after the reduction.
     * NOTE: This function will destroy the ring structure of the point cloud
     *
     * @param cloud point cloud message that contains data points from the lidar
     */
    void reduction_filter_average(fastsense::msg::PointCloudPtrStamped& cloud);

    /**
     * @brief Reduces the PointCloud with Voxel reduction
     *
     * This function reduces the amount of PointCloud points. It achieves this by merging together points which belong to the same voxel (given by MAP_RESOLUTION)
     * and replacing them by one point that has the same coordinates as the voxel center. So every voxel contains at most one point after the reduction.
     * NOTE: This function will destroy the ring structure of the point cloud
     *
     * @param cloud point cloud message that contains data points from the lidar
     */
    void reduction_filter_voxel_center(fastsense::msg::PointCloudPtrStamped& cloud);

    /**
     * @brief Reduces the PointCloud with Voxel reduction
     *
     * This function reduces the amount of PointCloud points. It achieves this by throwing out all points inside the voxel except one randomly chosen one.
     * So every voxel contains at most one point after the reduction.
     * NOTE: This function will destroy the ring structure of the point cloud
     *
     * @param cloud point cloud message that contains data points from the lidar
     */
    void reduction_filter_random_point(fastsense::msg::PointCloudPtrStamped& cloud);

    /**
     * @brief Reduces the PointCloud with Voxel reduction
     *
     * This function reduces the amount of PointCloud points. It achieves this by merging together points which belong to the same voxel (given by MAP_RESOLUTION)
     * and replacing them by the one point that is closest to the voxel center. So every voxel contains at most one point after the reduction.
     * NOTE: This function will destroy the ring structure of the point cloud
     *
     * @param cloud point cloud message that contains data points from the lidar
     */
    void reduction_filter_closest(fastsense::msg::PointCloudPtrStamped& cloud);

    /**
     * @brief Applies a Median Filter to the PointCloud
     * 
     * This function applies a median filter to the input point cloud. The median filter will be applied ring-wise. So every point in the point cloud
     * will be replaced by the median point (based of the euclidian distance to the origin) of its neighbours.
     *
     * @param cloud point cloud message that contains data points from the lidar
     * @param window_size determines the size of the neighbourhood that will be looked at to find the median point
     *        NOTE: window_size has to be odd
     */
    void median_filter(fastsense::msg::PointCloudPtrStamped& cloud, uint8_t window_size);

private:

    /**
     * @brief Finds the median value inside a std:vector
     *
     * @param medians std::vector of type ScanPoint
     */
    uint8_t median_from_array(std::vector<ScanPoint*> medians);

    const std::shared_ptr<PointCloudBuffer> send_buffer;
    bool send_original;
    bool send_preprocessed;
    float scale;
};

}