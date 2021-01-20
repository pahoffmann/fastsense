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
    void read_points(PointHW* scanPoints,
                     int numPoints,
                     const LocalMapHW& map,
                     int tau,
                     hls::stream<TSDFValueHW>& value_fifo,
                     hls::stream<PointHW>& index_fifo,
                     hls::stream<IntTuple>& bounds_fifo)
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

                TSDFValueHW tsdf;
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
                    tsdf.value = -tsdf.value;
                }

                tsdf.weight = WEIGHT_RESOLUTION - 1;

                if (tsdf.value < -weight_epsilon)
                {
                    tsdf.weight = WEIGHT_RESOLUTION * (tau + tsdf.value) / (tau - weight_epsilon);
                }

                if (tsdf.weight == 0)
                {
                    continue;
                }

                int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;
                IntTuple bounds;
                bounds.first = (proj.z - delta_z) / MAP_RESOLUTION;
                bounds.second = (proj.z + delta_z) / MAP_RESOLUTION;
                if (bounds.first < map_pos.z - map.sizeZ / 2)
                {
                    bounds.first = map_pos.z - map.sizeZ / 2;
                }
                if (bounds.second > map_pos.z + map.sizeZ / 2)
                {
                    bounds.second = map_pos.z + map.sizeZ / 2;
                }

                value_fifo << tsdf;
                index_fifo << index;
                bounds_fifo << bounds;
            }
        }
        value_fifo << TSDFValueHW{0, 0};
        index_fifo << PointHW();
        bounds_fifo << IntTuple{0, 0};
    }

    void update_tsdf(const LocalMapHW& map,
                     TSDFValueHW* new_entries,
                     hls::stream<TSDFValueHW>& value_fifo,
                     hls::stream<PointHW>& index_fifo,
                     hls::stream<IntTuple>& bounds_fifo)
    {
        TSDFValueHW value;
        PointHW index, old_index{0, 0, 0};
        IntTuple bounds{0, 0};
        int z = 1;

    update_loop:
        while (true)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR*128*2
#pragma HLS pipeline II=1
#pragma HLS dependence variable=new_entries inter false

            if (z > bounds.second)
            {
                old_index = index;

                value_fifo >> value;
                index_fifo >> index;
                bounds_fifo >> bounds;

                if (value.weight == 0)
                {
                    break;
                }
                z = bounds.first;
            }

            if (index.x != old_index.x || index.y != old_index.y)
            {
                int map_index = map.getIndex(index.x, index.y, z);
                TSDFValueHW entry = new_entries[map_index];

                if (entry.weight == 0 || hls_abs(value.value) < hls_abs(entry.value))
                {
                    new_entries[map_index] = value;
                }
                z++;
            }
            else
            {
                z = bounds.second + 1;
            }
        }
    }

    void sync_loop(
        TSDFValueHW* mapData,
        int start,
        int end,
        TSDFValueHW* new_entries,
        int max_weight)
    {
        for (int index = start; index < end; index++)
        {
#pragma HLS loop_tripcount min=8000000/SPLIT_FACTOR max=8000000/SPLIT_FACTOR
#pragma HLS pipeline II=1
#pragma HLS dependence variable=mapData inter false
#pragma HLS dependence variable=new_entries inter false

            TSDFValueHW map_entry = mapData[index];
            TSDFValueHW new_entry = new_entries[index];

            auto new_weight = map_entry.weight + new_entry.weight;

            if (new_weight)
            {
                map_entry.value = (map_entry.value * map_entry.weight + new_entry.value * new_entry.weight) / new_weight;

                if (new_weight > max_weight)
                {
                    new_weight = max_weight;
                }

                map_entry.weight = new_weight;

                mapData[index] = map_entry;
            }
        }
    }

    void tsdf_dataflower(PointHW* scanPoints0,
                         PointHW* scanPoints1,
                         PointHW* scanPoints2,
                         PointHW* scanPoints3,
                         int step,
                         int last_step,
                         TSDFValueHW* new_entries0,
                         TSDFValueHW* new_entries1,
                         TSDFValueHW* new_entries2,
                         TSDFValueHW* new_entries3,
                         const LocalMapHW& map,
                         int tau)
    {
#pragma HLS dataflow

        hls::stream<TSDFValueHW> value_fifo0;
        hls::stream<PointHW> index_fifo0;
        hls::stream<IntTuple> bounds_fifo0;
#pragma HLS stream depth=16 variable=value_fifo0
#pragma HLS stream depth=16 variable=index_fifo0
#pragma HLS stream depth=16 variable=bounds_fifo0
        read_points(scanPoints0, step,
                    map,
                    tau,
                    value_fifo0,
                    index_fifo0,
                    bounds_fifo0);
        update_tsdf(map,
                    new_entries0,
                    value_fifo0,
                    index_fifo0,
                    bounds_fifo0);

        hls::stream<TSDFValueHW> value_fifo1;
        hls::stream<PointHW> index_fifo1;
        hls::stream<IntTuple> bounds_fifo1;
#pragma HLS stream depth=16 variable=value_fifo1
#pragma HLS stream depth=16 variable=index_fifo1
#pragma HLS stream depth=16 variable=bounds_fifo1
        read_points(scanPoints1, step,
                    map,
                    tau,
                    value_fifo1,
                    index_fifo1,
                    bounds_fifo1);
        update_tsdf(map,
                    new_entries1,
                    value_fifo1,
                    index_fifo1,
                    bounds_fifo1);

        hls::stream<TSDFValueHW> value_fifo2;
        hls::stream<PointHW> index_fifo2;
        hls::stream<IntTuple> bounds_fifo2;
#pragma HLS stream depth=16 variable=value_fifo2
#pragma HLS stream depth=16 variable=index_fifo2
#pragma HLS stream depth=16 variable=bounds_fifo2
        read_points(scanPoints2, step,
                    map,
                    tau,
                    value_fifo2,
                    index_fifo2,
                    bounds_fifo2);
        update_tsdf(map,
                    new_entries2,
                    value_fifo2,
                    index_fifo2,
                    bounds_fifo2);

        hls::stream<TSDFValueHW> value_fifo3;
        hls::stream<PointHW> index_fifo3;
        hls::stream<IntTuple> bounds_fifo3;
#pragma HLS stream depth=16 variable=value_fifo3
#pragma HLS stream depth=16 variable=index_fifo3
#pragma HLS stream depth=16 variable=bounds_fifo3
        read_points(scanPoints3, last_step,
                    map,
                    tau,
                    value_fifo3,
                    index_fifo3,
                    bounds_fifo3);
        update_tsdf(map,
                    new_entries3,
                    value_fifo3,
                    index_fifo3,
                    bounds_fifo3);
    }

    void sync_looper(TSDFValueHW* mapData0,
                     TSDFValueHW* mapData1,
                     TSDFValueHW* mapData2,
                     TSDFValueHW* mapData3,
                     int end0,
                     int end1,
                     int end2,
                     int end3,
                     TSDFValueHW* new_entries0,
                     TSDFValueHW* new_entries1,
                     TSDFValueHW* new_entries2,
                     TSDFValueHW* new_entries3,
                     int max_weight)
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
                   TSDFValueHW* mapData0,
                   TSDFValueHW* mapData1,
                   TSDFValueHW* mapData2,
                   TSDFValueHW* mapData3,
                   int sizeX,   int sizeY,   int sizeZ,
                   int posX,    int posY,    int posZ,
                   int offsetX, int offsetY, int offsetZ,
                   TSDFValueHW* new_entries0,
                   TSDFValueHW* new_entries1,
                   TSDFValueHW* new_entries2,
                   TSDFValueHW* new_entries3,
                   int tau,
                   int max_weight)
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
                        tau);


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
