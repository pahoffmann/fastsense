/**
 * @author Marc Eisoldt
 * @author Malte Hillmann
 * @author Marcel Flottmann
 */

#include <map/local_map_hw.h>
#include <util/constants.h>
#include <util/point_hw.h>

#include <iostream>

#include <hls_stream.h>


using namespace fastsense::map;

constexpr int NUM_POINTS = 6000;
constexpr int SPLIT_FACTOR = 2;

struct QuadPoint
{
    int data[4];
    int& operator[](int index)
    {
        return data[index];
    }
};

extern "C"
{
    void read_points(PointHW* scanPoints,
                     int numPoints,
                     const LocalMapHW& map,
                     int tau,
                     const PointHW& up,
                     hls::stream<IntTuple>& value_fifo,
                     hls::stream<PointHW>& index_fifo,
                     hls::stream<std::pair<PointHW, PointHW>>& bounds_fifo,
                     hls::stream<IntTuple>& iter_steps_fifo)
    {
        PointHW map_pos{map.posX, map.posY, map.posZ};

        int weight_epsilon = tau / 10;
        int max_distance = (map.sizeX / 2 + map.sizeY / 2 + map.sizeZ / 2) * MAP_RESOLUTION;

    points_loop:
        for (int point_idx = 0; point_idx < numPoints; point_idx++)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR

            PointHW scan_point = scanPoints[point_idx];

            PointHW direction = scan_point - map_pos.to_mm();

            int distance = direction.norm();
            int distance_tau = distance + tau;
            if (distance_tau > max_distance)
            {
                distance_tau = max_distance;
            }

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

                IntTuple value;
                value.first = (scan_point - index.to_mm()).norm();
                if (value.first > tau)
                {
                    value.first = tau;
                }

                if (len > distance)
                {
                    value.first = -value.first;
                }

                value.second = WEIGHT_RESOLUTION;

                if (value.first < -weight_epsilon)
                {
                    value.second = WEIGHT_RESOLUTION * (tau + value.first) / (tau - weight_epsilon);
                }

                if (value.second == 0)
                {
                    continue;
                }

                int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;
                std::pair<PointHW, PointHW> bounds;
                IntTuple iter_steps;

                iter_steps.first = (delta_z * 2) / MAP_RESOLUTION + 1;
                iter_steps.second = delta_z / MAP_RESOLUTION;
                
                // bounds.first = (proj.z - delta_z) / MAP_RESOLUTION;
                // bounds.second = (proj.z + delta_z) / MAP_RESOLUTION;
                // if (bounds.first < map_pos.z - map.sizeZ / 2)
                // {
                //     bounds.first = map_pos.z - map.sizeZ / 2;
                // }
                // if (bounds.second > map_pos.z + map.sizeZ / 2)
                // {
                //     bounds.second = map_pos.z + map.sizeZ / 2;
                // }

                bounds.first = (proj - (up * delta_z) / MATRIX_RESOLUTION) / MAP_RESOLUTION;
                bounds.second = (proj + (up * delta_z) / MATRIX_RESOLUTION) / MAP_RESOLUTION;

                value_fifo << value;
                index_fifo << index;
                bounds_fifo << bounds;
                iter_steps_fifo << iter_steps;
            }
        }

        value_fifo << IntTuple{0, 0};
        index_fifo << PointHW();
        bounds_fifo << std::pair<PointHW, PointHW>{PointHW(), PointHW()};
        iter_steps_fifo << IntTuple(0, 0);
    }

    void update_tsdf(const LocalMapHW& map,
                     IntTuple* new_entries,
                     const PointHW& up,
                     hls::stream<IntTuple>& value_fifo,
                     hls::stream<PointHW>& index_fifo,
                     hls::stream<std::pair<PointHW, PointHW>>& bounds_fifo,
                     hls::stream<IntTuple>& iter_steps_fifo)
    {
        IntTuple value;
        PointHW index, old_index{0, 0, 0};
        std::pair<PointHW, PointHW> bounds{PointHW(), PointHW()};
        IntTuple iter_steps{0, 0};
        //PointHW z;
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

                value_fifo >> value;
                index_fifo >> index;
                bounds_fifo >> bounds;
                iter_steps_fifo >> iter_steps;

                if (value.second == 0)
                {
                    break;
                }
                
                //z = bounds.first;
                step = 0;
                //mid = (bounds.second + bounds.first) / 2;
            }

            if (index.x != old_index.x || index.y != old_index.y)
            {
                index = bounds.first + ((up * step) / MATRIX_RESOLUTION);

                int map_index = map.getIndex(index.x, index.y, index.z);
                IntTuple entry = new_entries[map_index];
                IntTuple tmp_value = value;

                if (entry.second <= 0 || hls_abs(value.first) < hls_abs(entry.first))
                {
                    if (step != iter_steps.second)
                    {
                        tmp_value.second *= -1;
                    }

                    new_entries[map_index] = tmp_value;
                }

                step++;
            }
            else
            {
                step = iter_steps.first + 1;
            }
        }
    }

    void tsdf_dataflow(PointHW* scanPoints,
                       int numPoints,
                       const LocalMapHW& map,
                       IntTuple* new_entries,
                       int tau,
                       const PointHW& up)
    {
#pragma HLS dataflow
        hls::stream<IntTuple> value_fifo;
        hls::stream<PointHW> index_fifo;
        //hls::stream<IntTuple> bounds_fifo;
        hls::stream<std::pair<PointHW, PointHW>> bounds_fifo;
        hls::stream<IntTuple> iter_steps_fifo;
#pragma HLS stream depth=16 variable=value_fifo
#pragma HLS stream depth=16 variable=index_fifo
#pragma HLS stream depth=16 variable=bounds_fifo
#pragma HLS stream depth=16 variable=iter_steps_fifo

        read_points(scanPoints, numPoints,
                    map,
                    tau,
                    up,
                    value_fifo,
                    index_fifo,
                    bounds_fifo,
                    iter_steps_fifo);

        update_tsdf(map,
                    new_entries,
                    up,
                    value_fifo,
                    index_fifo,
                    bounds_fifo,
                    iter_steps_fifo);
    }

    void sync_loop(
        IntTuple* mapData,
        int start,
        int end,
        IntTuple* new_entries,
        int max_weight)
    {
        for (int index = start; index < end; index++)
        {
#pragma HLS loop_tripcount min=8000000/SPLIT_FACTOR max=8000000/SPLIT_FACTOR
#pragma HLS pipeline II=1
#pragma HLS dependence variable=mapData inter false
#pragma HLS dependence variable=new_entries inter false

            IntTuple map_entry = mapData[index];
            IntTuple new_entry = new_entries[index];

            int new_weight = map_entry.second + new_entry.second;

            if (new_entry.second > 0 && map_entry.second > 0)
            {   
                map_entry.first = (map_entry.first * map_entry.second + new_entry.first * new_entry.second) / new_weight;

                if (new_weight > max_weight)
                {
                    new_weight = max_weight;
                }

                map_entry.second = new_weight;
            }
            else if (new_entry.second != 0 && map_entry.second <= 0)
            {
                map_entry.first = new_entry.first;
                map_entry.second = new_entry.second;
            }

            mapData[index] = map_entry;
        }
    }

    void tsdf_dataflower(PointHW* scanPoints0,
                         PointHW* scanPoints1,
                         int step,
                         int last_step,
                         IntTuple* new_entries0,
                         IntTuple* new_entries1,
                         const LocalMapHW& map,
                         int tau,
                         const PointHW& up)
    {
#pragma HLS dataflow

        tsdf_dataflow(scanPoints0 + 0 * step, step,
                      map, new_entries0, tau, up);

        tsdf_dataflow(scanPoints1 + 1 * step, last_step,
                      map, new_entries1, tau, up);
    }

    void sync_looper(IntTuple* mapData0,
                     IntTuple* mapData1,
                     int step,
                     int numPoints,
                     IntTuple* new_entries0,
                     IntTuple* new_entries1,
                     int max_weight)
    {
#pragma HLS dataflow
        sync_loop(mapData0, step * 0, step * (0 + 1), new_entries0, max_weight);
        sync_loop(mapData1, step * 1, numPoints, new_entries1, max_weight);
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
                   int numPoints,
                   IntTuple* mapData0,
                   IntTuple* mapData1,
                   int sizeX,   int sizeY,   int sizeZ,
                   int posX,    int posY,    int posZ,
                   int offsetX, int offsetY, int offsetZ,
                   IntTuple* new_entries0,
                   IntTuple* new_entries1,
                   int tau,
                   int max_weight,
                   int up_x, int up_y, int up_z)
    {
#pragma HLS INTERFACE m_axi port=scanPoints0  offset=slave bundle=scan0mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=scanPoints1  offset=slave bundle=scan1mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=mapData0     offset=slave bundle=map0mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=mapData1     offset=slave bundle=map1mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries0 offset=slave bundle=entry0mem latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries1 offset=slave bundle=entry1mem latency=22 depth=18491

        PointHW up(up_x, up_y, up_z);


        LocalMapHW map{sizeX,   sizeY,   sizeZ,
                       posX,    posY,    posZ,
                       offsetX, offsetY, offsetZ};

        int step = numPoints / SPLIT_FACTOR + 1;
        int last_step = numPoints - (SPLIT_FACTOR - 1) * step;
        tsdf_dataflower(scanPoints0,
                        scanPoints1,
                        step,
                        last_step,
                        new_entries0,
                        new_entries1,
                        map,
                        tau,
                        up);

        int total_size = sizeX * sizeY * sizeZ;
        int sync_step = total_size / SPLIT_FACTOR + 1;

        sync_looper(mapData0,
                    mapData1,
                    sync_step,
                    total_size,
                    new_entries0,
                    new_entries1,
                    max_weight);
    }
}
