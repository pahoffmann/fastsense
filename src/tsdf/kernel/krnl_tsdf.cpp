/**
 * @author Marc Eisoldt
 * @author Malte Hillmann
 * @author Marcel Flottmann
 */

#include <map/local_map_hw.h>
#include <util/constants.h>
#include <util/point_hw.h>
#include <util/tsdf_hw.h>

#include <iostream>

#include <hls_stream.h>

using namespace fastsense::map;

constexpr int NUM_POINTS = 6000;
constexpr int SPLIT_FACTOR = 4;

extern "C"
{
    /**
     * @brief generates data for update_tsdf
     *
     * executes raymarching from the Scanner to the Point, calculates the TSDF Value for each Cell
     * and sends that data to update_tsdf for interpolation
     *
     * @param scanPoints an array of Points
     * @param numPoints length of scanPoints
     * @param map metadata of the LocalMap. Needed for Scanner position and Map size
     * @param tau the truncation distance for tsdf values
     * @param up up-vector for the orientation of the Scanner
     * @param value_fifo fifo to send the calculated tsdf value to update_tsdf
     * @param index_fifo fifo to send the current cell to update_tsdf
     * @param bounds_fifo fifo to send (starting point, interpolation vector) to update_tsdf
     * @param iter_steps_fifo fifo to send (number of interpolation steps, index of current cell) to update_tsdf
     */
    void read_points(PointHW* scanPoints,
                     int numPoints,
                     const LocalMapHW& map,
                     TSDFEntryHW::ValueType tau,
                     const PointHW& up,
                     hls::stream<TSDFEntryHW>& value_fifo,
                     hls::stream<PointHW>& index_fifo,
                     hls::stream<std::pair<PointHW, PointArith>>& bounds_fifo,
                     hls::stream<std::pair<int, int>>& iter_steps_fifo)
    {
        // Position of the Scanner
        PointHW map_pos{map.posX, map.posY, map.posZ};

        // grace period around the Point before the weight of a Point decreases
        TSDFEntryHW::ValueType weight_epsilon = tau / 10;

        // rough estimate for the maximum raymarching distance
        int max_distance = (map.sizeX / 2 + map.sizeY / 2 + map.sizeZ / 2) * MAP_RESOLUTION;

    points_loop:
        for (int point_idx = 0; point_idx < numPoints; point_idx++)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR

            PointHW scan_point = scanPoints[point_idx];

            PointHW direction = scan_point - map_pos.to_mm();

            int distance = direction.norm();
            // end of raymarching: truncation distance behind the Point
            int distance_tau = distance + tau;

            auto normed_direction_vector = (PointArith(direction.x, direction.y, direction.z) * MATRIX_RESOLUTION) / distance;

            // interpolation vector should be perpendicular to the direction vector and should be on the same Plane as the up-vector
            // => calculate adjusted-up-vector = direction X right, with right-vector = direction X original-up-vector
            auto interpolation_vector = (normed_direction_vector.cross(normed_direction_vector.cross(PointArith(up.x, up.y, up.z)) / MATRIX_RESOLUTION));

            auto normed_interpolation_vector = (interpolation_vector * MATRIX_RESOLUTION) / interpolation_vector.norm();

            if (distance_tau > max_distance)
            {
                distance_tau = max_distance;
            }

            // the main Raymarching Loop
            // start at MAP_RESOLUTION to avoid problems with the Scanner pos
            // step in half-cell-size steps to possibly catch multiple cells on a slope
            //     this would lead to some cells being visited twice, which is filtered by update_tsdf
            //     the filter used to be inside this function, but that would prevent pipelining with II=1,
            //     because the rest of the loop would then depend on that calculation and Vitis says no
        tsdf_loop:
            for (int len = MAP_RESOLUTION; len <= distance_tau; len += MAP_RESOLUTION / 2)
            {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount min=0 max=128

                PointHW proj = map_pos.to_mm() + direction * len / distance;
                PointHW index = proj.to_map();

                if (!map.in_bounds(index.x, index.y, index.z))
                {
                    continue;
                }

                TSDFEntryHW tsdf;
                auto value = (scan_point - index.to_mm()).norm();
                if (value > tau)
                {
                    tsdf.value = tau;
                }
                else
                {
                    tsdf.value = value;
                }

                if (len > distance)
                {
                    // tsdf is negative behind the Point
                    tsdf.value = -tsdf.value;
                }

                tsdf.weight = WEIGHT_RESOLUTION;

                // weighting function:
                // weight = 1 (aka WEIGHT_RESOLUTION) from Scanner to Point
                // linear descent to 0 after the Point, starting at Point + weight_epsilon
                if (tsdf.value < -weight_epsilon)
                {
                    tsdf.weight = WEIGHT_RESOLUTION * (tau + tsdf.value) / (tau - weight_epsilon);
                }

                if (tsdf.weight == 0)
                {
                    continue;
                }

                // delta_z == how many cells should be interpolated to fill the area between rings
                int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;
                std::pair<PointHW, PointArith> bounds;
                std::pair<int, int> iter_steps;

                iter_steps.first = (delta_z * 2) / MAP_RESOLUTION + 1;
                iter_steps.second = delta_z / MAP_RESOLUTION;

                auto lowest = PointArith(proj.x, proj.y, proj.z) - ((normed_interpolation_vector * delta_z) / MATRIX_RESOLUTION);
                bounds.first = PointHW(lowest.x, lowest.y, lowest.z); // / MAP_RESOLUTION;
                bounds.second = normed_interpolation_vector;

                value_fifo << tsdf;
                index_fifo << index;
                bounds_fifo << bounds;
                iter_steps_fifo << iter_steps;
            }
        }
        // send a final dummy message to terminate the update_loop
        value_fifo << TSDFEntryHW{0, 0};
        index_fifo << PointHW();
        bounds_fifo << std::pair<PointHW, PointArith> {PointHW(), PointArith()};
        iter_steps_fifo << std::pair<int, int>(0, 0);
    }

    /**
     * @brief interpolates a tsdf-value along a path to fill the area between rings
     * 
     * @param map metadata of the LocalMap
     * @param new_entries temporary local map for newly generated values
     * @param value_fifo fifo to receive the calculated tsdf value from read_points
     * @param index_fifo fifo to receive the current cell from read_points
     * @param bounds_fifo fifo to receive (starting point, interpolation vector) from read_points
     * @param iter_steps_fifo fifo to receive (number of interpolation steps, index of current cell) from read_points
     */
    void update_tsdf(const LocalMapHW& map,
                     TSDFEntryHW* new_entries,
                     hls::stream<TSDFEntryHW>& value_fifo,
                     hls::stream<PointHW>& index_fifo,
                     hls::stream<std::pair<PointHW, PointArith>>& bounds_fifo,
                     hls::stream<std::pair<int, int>>& iter_steps_fifo)
    {
        TSDFEntryHW new_value;
        PointHW index{0, 0, 0}, old_index{0, 0, 0};
        std::pair<PointHW, PointArith> bounds{PointHW(), PointArith()};
        std::pair<int, int> iter_steps{0, 0};
        int step = 1;

    update_loop:
        while (true)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR*128*2
#pragma HLS pipeline II=1
#pragma HLS dependence variable=new_entries inter false

            if (step > iter_steps.first)
            {
                old_index = index;

                value_fifo >> new_value;
                index_fifo >> index;
                bounds_fifo >> bounds;
                iter_steps_fifo >> iter_steps;

                // the weight is only 0 in the dummy message
                if (new_value.weight == 0)
                {
                    break;
                }
                step = 0;
            }

            // tsdf_loop iterates in half-MAP_RESOLUTION steps => might visit the same cell twice
            // this check used to be done at the beginning of the tsdf_loop, but that leads to timing violations
            if (index != old_index)
            {
                auto index_arith = PointArith(bounds.first.x, bounds.first.y, bounds.first.z) + ((bounds.second * (step * MAP_RESOLUTION)) / MATRIX_RESOLUTION);
                index = PointHW(index_arith.x, index_arith.y, index_arith.z) / MAP_RESOLUTION;

                int map_index = map.getIndex(index.x, index.y, index.z);

                TSDFEntryHW old_entry = new_entries[map_index];

                // interpolated values have a negative weight
                bool old_is_interpolated = old_entry.weight <= 0;
                bool current_is_interpolated = step != iter_steps.second;
                bool current_is_better = hls_abs(new_value.value) < hls_abs(old_entry.value) || old_entry.weight == 0;

                if ((current_is_better || old_is_interpolated) && map.in_bounds(index.x, index.y, index.z))
                {
                    TSDFEntryHW tmp_value = old_entry;
                    // we always want the smallest tsdf value in each cell, even from interpolated values
                    if (current_is_better)
                    {
                        tmp_value.value = new_value.value;
                    }

                    if (old_is_interpolated)
                    {
                        // weight is only negative when old an new are both interpolated
                        tmp_value.weight = new_value.weight * (current_is_interpolated ? -1 : 1);
                    }

                    new_entries[map_index] = tmp_value;
                }

                step++;
            }
            else
            {
                // The current Point has already been processed
                // => skip interpolation and take next Point from fifo
                step = iter_steps.first + 1;
            }
        }
    }

    /**
     * @brief update the local map with the new values using floating average
     * 
     * this function is called multiple times in parallel with different start and end indices
     * 
     * @param mapData the local map
     * @param start the starting index of the range of this instance
     * @param end the end index of the range of this instance
     * @param new_entries the temporary map with the new values
     * @param max_weight the maximum weight for the floating average
     */
    void sync_loop(
        TSDFEntryHW* mapData,
        int start,
        int end,
        TSDFEntryHW* new_entries,
        TSDFEntryHW::WeightType max_weight)
    {
        // Update the current map based on the new generated entries
        for (int index = start; index < end; index++)
        {
#pragma HLS loop_tripcount min=8000000/SPLIT_FACTOR max=8000000/SPLIT_FACTOR
#pragma HLS pipeline II=1
#pragma HLS dependence variable=mapData inter false
#pragma HLS dependence variable=new_entries inter false

            TSDFEntryHW map_entry = mapData[index];
            TSDFEntryHW new_entry = new_entries[index];

            int new_weight = map_entry.weight + new_entry.weight;

            // Averaging is only performed based on real measured entries and not on interpolated ones
            if (new_entry.weight > 0 && map_entry.weight > 0)
            {
                map_entry.value = (map_entry.value * map_entry.weight + new_entry.value * new_entry.weight) / new_weight;

                // Upper bound for the total weight. Ensures, that later updates have still an impact. 
                if (new_weight > max_weight)
                {
                    new_weight = max_weight;
                }

                map_entry.weight = new_weight;
            }
            // An interpolated value will always be overwritten by a new one. Real values are always preferred
            else if (new_entry.weight != 0 && map_entry.weight <= 0)
            {
                map_entry.value = new_entry.value;
                map_entry.weight = new_entry.weight;
            }

            mapData[index] = map_entry;
        }
    }

    void tsdf_dataflower(PointHW* scanPoints0,
                         PointHW* scanPoints1,
                         PointHW* scanPoints2,
                         PointHW* scanPoints3,
                         int step,
                         int last_step,
                         TSDFEntryHW* new_entries0,
                         TSDFEntryHW* new_entries1,
                         TSDFEntryHW* new_entries2,
                         TSDFEntryHW* new_entries3,
                         const LocalMapHW& map,
                         TSDFEntryHW::ValueType tau,
                         const PointHW& up)
    {
#pragma HLS dataflow

        hls::stream<TSDFEntryHW> value_fifo0;
        hls::stream<PointHW> index_fifo0;
        hls::stream<std::pair<PointHW, PointArith>> bounds_fifo0;
        hls::stream<std::pair<int, int>> iter_steps_fifo0;
#pragma HLS stream depth=16 variable=value_fifo0
#pragma HLS stream depth=16 variable=index_fifo0
#pragma HLS stream depth=16 variable=bounds_fifo0
        read_points(scanPoints0, step,
                    map, tau, up,
                    value_fifo0,
                    index_fifo0,
                    bounds_fifo0,
                    iter_steps_fifo0);
        update_tsdf(map,
                    new_entries0,
                    value_fifo0,
                    index_fifo0,
                    bounds_fifo0,
                    iter_steps_fifo0);

        hls::stream<TSDFEntryHW> value_fifo1;
        hls::stream<PointHW> index_fifo1;
        hls::stream<std::pair<PointHW, PointArith>> bounds_fifo1;
        hls::stream<std::pair<int, int>> iter_steps_fifo1;
#pragma HLS stream depth=16 variable=value_fifo1
#pragma HLS stream depth=16 variable=index_fifo1
#pragma HLS stream depth=16 variable=bounds_fifo1
        read_points(scanPoints1, step,
                    map, tau, up,
                    value_fifo1,
                    index_fifo1,
                    bounds_fifo1,
                    iter_steps_fifo1);
        update_tsdf(map,
                    new_entries1,
                    value_fifo1,
                    index_fifo1,
                    bounds_fifo1,
                    iter_steps_fifo1);

        hls::stream<TSDFEntryHW> value_fifo2;
        hls::stream<PointHW> index_fifo2;
        hls::stream<std::pair<PointHW, PointArith>> bounds_fifo2;
        hls::stream<std::pair<int, int>> iter_steps_fifo2;
#pragma HLS stream depth=16 variable=value_fifo2
#pragma HLS stream depth=16 variable=index_fifo2
#pragma HLS stream depth=16 variable=bounds_fifo2
        read_points(scanPoints2, step,
                    map, tau, up,
                    value_fifo2,
                    index_fifo2,
                    bounds_fifo2,
                    iter_steps_fifo2);
        update_tsdf(map,
                    new_entries2,
                    value_fifo2,
                    index_fifo2,
                    bounds_fifo2,
                    iter_steps_fifo2);

        hls::stream<TSDFEntryHW> value_fifo3;
        hls::stream<PointHW> index_fifo3;
        hls::stream<std::pair<PointHW, PointArith>> bounds_fifo3;
        hls::stream<std::pair<int, int>> iter_steps_fifo3;
#pragma HLS stream depth=16 variable=value_fifo3
#pragma HLS stream depth=16 variable=index_fifo3
#pragma HLS stream depth=16 variable=bounds_fifo3
        read_points(scanPoints3, last_step,
                    map, tau, up,
                    value_fifo3,
                    index_fifo3,
                    bounds_fifo3,
                    iter_steps_fifo3);
        update_tsdf(map,
                    new_entries3,
                    value_fifo3,
                    index_fifo3,
                    bounds_fifo3,
                    iter_steps_fifo3);
    }

    void sync_looper(TSDFEntryHW* mapData0,
                     TSDFEntryHW* mapData1,
                     TSDFEntryHW* mapData2,
                     TSDFEntryHW* mapData3,
                     int end0,
                     int end1,
                     int end2,
                     int end3,
                     TSDFEntryHW* new_entries0,
                     TSDFEntryHW* new_entries1,
                     TSDFEntryHW* new_entries2,
                     TSDFEntryHW* new_entries3,
                     TSDFEntryHW::WeightType max_weight)
    {
#pragma HLS dataflow
        sync_loop(mapData0, 0, end0, new_entries0, max_weight);
        sync_loop(mapData1, end0, end1, new_entries1, max_weight);
        sync_loop(mapData2, end1, end2, new_entries2, max_weight);
        sync_loop(mapData3, end2, end3, new_entries3, max_weight);
    }

    /**
     * @brief Hardware implementation of the TSDF generation and update algorithm using bresenham
     *
     * @param scanPoints0 First point reference from which the TSDF data should be calculated
     * @param scanPoints1 Second point reference from which the TSDF data should be calculated
     * @param numPoints Number of points which should be
     * @param mapData0 First map reference which should be used for the update
     * @param mapData1 Second map reference which should be used for the update
     * @param sizeX Number of map cells in x direction
     * @param sizeY Number of map cells in y direction
     * @param sizeZ Number of map cells in z direction
     * @param posX X coordinate of the scanner
     * @param posY Y coordinate of the scanner
     * @param posZ Z coordinate of the scanner
     * @param offsetX X offset of the local map
     * @param offsetY Y offset of the local map
     * @param offsetZ Z offset of the local map
     * @param new_entries0 First reference to the temporal buffer for the calculated TSDF values
     * @param new_entries1 Second reference to the temporal buffer for the calculated TSDF values
     * @param tau Truncation distance for the TSDF values (in map resolution)
     * @param max_weight Maximum for the weight of the map entries
     */
    void krnl_tsdf(PointHW* scanPoints0,
                   PointHW* scanPoints1,
                   PointHW* scanPoints2,
                   PointHW* scanPoints3,
                   int numPoints,
                   TSDFEntryHW* mapData0,
                   TSDFEntryHW* mapData1,
                   TSDFEntryHW* mapData2,
                   TSDFEntryHW* mapData3,
                   int sizeX,   int sizeY,   int sizeZ,
                   int posX,    int posY,    int posZ,
                   int offsetX, int offsetY, int offsetZ,
                   TSDFEntryHW* new_entries0,
                   TSDFEntryHW* new_entries1,
                   TSDFEntryHW* new_entries2,
                   TSDFEntryHW* new_entries3,
                   TSDFEntryHW::ValueType tau,
                   TSDFEntryHW::WeightType max_weight,
                   int up_x, int up_y, int up_z)
    {
#pragma HLS INTERFACE m_axi port=scanPoints0  offset=slave bundle=scan0mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=scanPoints1  offset=slave bundle=scan1mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=scanPoints2  offset=slave bundle=scan2mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=scanPoints3  offset=slave bundle=scan3mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=mapData0     offset=slave bundle=map0mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=mapData1     offset=slave bundle=map1mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=mapData2     offset=slave bundle=map2mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=mapData3     offset=slave bundle=map3mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries0 offset=slave bundle=entry0mem latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries1 offset=slave bundle=entry1mem latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries2 offset=slave bundle=entry2mem latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries3 offset=slave bundle=entry3mem latency=22 depth=18491

        PointHW up(up_x, up_y, up_z);


        LocalMapHW map{sizeX,   sizeY,   sizeZ,
                       posX,    posY,    posZ,
                       offsetX, offsetY, offsetZ};

        int step = numPoints / SPLIT_FACTOR;
        int last_step = numPoints - (SPLIT_FACTOR - 1) * step;
        tsdf_dataflower(scanPoints0,
                        scanPoints1 + 1 * step,
                        scanPoints2 + 2 * step,
                        scanPoints3 + 3 * step,
                        step,
                        last_step,
                        new_entries0,
                        new_entries1,
                        new_entries2,
                        new_entries3,
                        map,
                        tau,
                        up);

        int total_size = sizeX * sizeY * sizeZ;
        int sync_step = total_size / SPLIT_FACTOR + 1;

        sync_looper(mapData0,
                    mapData1,
                    mapData2,
                    mapData3,
                    1 * sync_step,
                    2 * sync_step,
                    3 * sync_step,
                    total_size,
                    new_entries0,
                    new_entries1,
                    new_entries2,
                    new_entries3,
                    max_weight);
    }
}
