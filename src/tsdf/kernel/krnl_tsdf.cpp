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

using IntTuple = std::pair<int, int>;
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
                     int tau,
                     PointHW map_pos,
                     hls::stream<PointHW>& point_fifo,
                     hls::stream<IntTuple>& distance_fifo,
                     hls::stream<QuadPoint>& direction_fifo,
                     hls::stream<QuadPoint>& abs_direction_fifo,
                     hls::stream<uint8_t>& major_axis_fifo)
    {
    points_loop:
        for (int point_idx = 0; point_idx < numPoints; point_idx++)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR
#pragma HLS pipeline II=1
            PointHW p = scanPoints[(point_idx * 157) % numPoints];
            int dist = p.norm();
            int delta_z = dz_per_distance * dist / MATRIX_RESOLUTION;
            int lower_z = (p.z - delta_z) / MAP_RESOLUTION;
            int upper_z = (p.z + delta_z) / MAP_RESOLUTION;

            PointHW dir = p.to_map() - map_pos;
            QuadPoint direction, abs_direction;
#pragma HLS array_partition complete variable=direction.data
            direction[0] = dir.x;
            direction[1] = dir.y;
            direction[2] = lower_z;
            direction[3] = upper_z;
            for (uint8_t i = 0; i < 4; i++)
            {
#pragma HLS unroll
                abs_direction[i] = hls_abs(direction[i]);
            }

            uint8_t major_axis;
            IntTuple distance;
            if (abs_direction[0] >= abs_direction[1] && abs_direction[0] >= abs_direction[2] && abs_direction[0] >= abs_direction[3])
            {
                major_axis = 0;
                distance.first = abs_direction[0];
            }
            else if (abs_direction[1] >= abs_direction[0] && abs_direction[1] >= abs_direction[2] && abs_direction[1] >= abs_direction[3])
            {
                major_axis = 1;
                distance.first = abs_direction[1];
            }
            else if (abs_direction[2] >= abs_direction[0] && abs_direction[2] >= abs_direction[1] && abs_direction[2] >= abs_direction[3])
            {
                major_axis = 2;
                distance.first = abs_direction[2];
            }
            else
            {
                major_axis = 3;
                distance.first = abs_direction[3];
            }
            distance.second = distance.first + tau / MAP_RESOLUTION;
            point_fifo << p;
            distance_fifo << distance;
            direction_fifo << direction;
            abs_direction_fifo << abs_direction;
            major_axis_fifo << major_axis;
        }
    }

    void update_tsdf(int numPoints,
                     int sizeX,   int sizeY,   int sizeZ,
                     int posX,    int posY,    int posZ,
                     int offsetX, int offsetY, int offsetZ,
                     IntTuple* new_entries,
                     int tau,
                     hls::stream<PointHW>& point_fifo,
                     hls::stream<IntTuple>& distance_fifo,
                     hls::stream<QuadPoint>& direction_fifo,
                     hls::stream<QuadPoint>& abs_direction_fifo,
                     hls::stream<uint8_t>& major_axis_fifo)
    {
        fastsense::map::LocalMapHW map{sizeX,   sizeY,   sizeZ,
                                       posX,    posY,    posZ,
                                       offsetX, offsetY, offsetZ};
        PointHW map_pos{posX, posY, posZ};

        int weight_epsilon = tau / 10;

        IntTuple distance;
        int current_distance;

        PointHW current_point;

        uint8_t major_axis;
        QuadPoint increment;
        QuadPoint error;
        QuadPoint current_cell;
        QuadPoint direction;
        QuadPoint abs_direction;
        QuadPoint abs_direction_double;
#pragma HLS array_partition complete variable=increment.data
#pragma HLS array_partition complete variable=error.data
#pragma HLS array_partition complete variable=current_cell.data
#pragma HLS array_partition complete variable=direction.data
#pragma HLS array_partition complete variable=abs_direction.data
#pragma HLS array_partition complete variable=abs_direction_double.data

        int interpolate_z;
        int interpolate_z_end;

        int point_idx = 0;

        bool load_new_point = true;
    tsdf_loop:
        while (point_idx < numPoints)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR*128*2
#pragma HLS pipeline II=1

            // Load point and init Bresenham
            if (load_new_point)
            {
                PointHW dir, abs_dir;
                point_fifo >> current_point;
                distance_fifo >> distance;
                direction_fifo >> direction;
                abs_direction_fifo >> abs_direction;
                major_axis_fifo >> major_axis;

                current_distance = 0;
                current_cell[0] = posX;
                current_cell[1] = posY;
                current_cell[2] = current_cell[3] = posZ;

                for (uint8_t i = 0; i < 4; i++)
                {
#pragma HLS unroll
                    abs_direction_double[i] = abs_direction[i] * 2;
                }
                for (uint8_t i = 0; i < 4; i++)
                {
#pragma HLS unroll
                    if (i != major_axis)
                    {
                        error[i] = abs_direction_double[i] - abs_direction[major_axis];
                    }
                    else
                    {
                        error[i] = 0;
                    }

                    increment[i] = direction[i] < 0 ? -1 : 1;
                }

                interpolate_z = current_cell[2];
                interpolate_z_end = current_cell[3];

                load_new_point = false;
            }
            else if (current_distance >= distance.second)
            {
                // this is in 'else' to convince Vitis that there is at least one iteration per Point and
                // the condition won't be checked in the same iteration in which distance_tau is read from fifo

                point_idx++;
                load_new_point = true;
            }

            // Calculate and update TSDF value
            PointHW cell(current_cell[0], current_cell[1], (current_cell[2] + current_cell[3]) / 2);
            int tsdf_value = (current_point - cell.to_mm()).norm();
            if (tsdf_value > tau)
            {
                tsdf_value = tau;
            }

            if (current_distance > distance.first)
            {
                tsdf_value = -tsdf_value;
            }

            int weight = WEIGHT_RESOLUTION;

            if (tsdf_value < -weight_epsilon)
            {
                weight = WEIGHT_RESOLUTION * (tau + tsdf_value) / (tau - weight_epsilon);
            }

            if (weight != 0 && map.in_bounds(current_cell[0], current_cell[1], interpolate_z))
            {
#pragma HLS dependence variable=new_entries inter false
                int index = map.getIndex(current_cell[0], current_cell[1], interpolate_z);
                IntTuple entry = new_entries[index];

                if (entry.second == 0 || hls_abs(tsdf_value) < hls_abs(entry.first))
                {
                    IntTuple new_entry;
                    new_entry.first = tsdf_value;
                    new_entry.second = weight;
                    new_entries[index] = new_entry;
                }
            }

            // Update current_cell with Bresenham and interpolation
            if (interpolate_z == interpolate_z_end)
            {
                if (error[0] <= 0 && error[1] <= 0 && error[2] <= 0 && error[3] <= 0)
                {
                    current_cell[major_axis] += increment[major_axis];
                    current_distance++;
                    for (uint8_t i = 0; i < 4; i++)
                    {
#pragma HLS unroll
                        if (i != major_axis)
                        {
                            error[i] += abs_direction_double[i];
                        }
                    }
                }
                else
                {
                    // TODO: Make this part smarter?:
                    //       - never change x and y simultaneously
                    //       - if only lower_z or only upper_z changed, set interpolate_z accordingly
                    for (uint8_t i = 0; i < 4; i++)
                    {
#pragma HLS unroll
                        if (i != major_axis && error[i] > 0)
                        {
                            current_cell[i] += increment[i];
                            error[i] -= abs_direction_double[major_axis];
                        }
                    }
                }

                interpolate_z = current_cell[2];
                interpolate_z_end = current_cell[3];
            }
            else
            {
                interpolate_z++;
            }
        }
    }

    void tsdf_dataflow(PointHW* scanPoints,
                       int numPoints,
                       int sizeX,   int sizeY,   int sizeZ,
                       int posX,    int posY,    int posZ,
                       int offsetX, int offsetY, int offsetZ,
                       IntTuple* new_entries,
                       int tau)
    {
#pragma HLS dataflow
        hls::stream<PointHW> point_fifo;
        hls::stream<IntTuple> distance_fifo;
        hls::stream<QuadPoint> direction_fifo;
        hls::stream<QuadPoint> abs_direction_fifo;
        hls::stream<uint8_t> major_axis_fifo;
#pragma HLS stream depth=16 variable=point_fifo
#pragma HLS stream depth=16 variable=distance_fifo
#pragma HLS stream depth=16 variable=direction_fifo
#pragma HLS stream depth=16 variable=abs_direction_fifo
#pragma HLS stream depth=16 variable=major_axis_fifo

        PointHW map_pos{posX, posY, posZ};

        read_points(scanPoints, numPoints, tau,
                    map_pos,
                    point_fifo,
                    distance_fifo,
                    direction_fifo,
                    abs_direction_fifo,
                    major_axis_fifo);

        update_tsdf(numPoints,
                    sizeX,   sizeY,   sizeZ,
                    posX,    posY,    posZ,
                    offsetX, offsetY, offsetZ,
                    new_entries, tau,
                    point_fifo,
                    distance_fifo,
                    direction_fifo,
                    abs_direction_fifo,
                    major_axis_fifo);
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

            if (new_weight)
            {
                map_entry.first = (map_entry.first * map_entry.second + new_entry.first * new_entry.second) / new_weight;

                if (new_weight > max_weight)
                {
                    new_weight = max_weight;
                }

                map_entry.second = new_weight;

                mapData[index] = map_entry;
            }
        }
    }

    void tsdf_dataflower(
        PointHW* scanPoints0,
        PointHW* scanPoints1,
        int step,
        int last_step,
        IntTuple* new_entries0,
        IntTuple* new_entries1,
        int sizeX,   int sizeY,   int sizeZ,
        int posX,    int posY,    int posZ,
        int offsetX, int offsetY, int offsetZ,
        int tau)
    {
#pragma HLS dataflow

        tsdf_dataflow(scanPoints0 + 0 * step, step,
                      sizeX,   sizeY,   sizeZ,
                      posX,    posY,    posZ,
                      offsetX, offsetY, offsetZ,
                      new_entries0, tau);

        tsdf_dataflow(scanPoints1 + 1 * step, last_step,
                      sizeX,   sizeY,   sizeZ,
                      posX,    posY,    posZ,
                      offsetX, offsetY, offsetZ,
                      new_entries1, tau);
    }

    void sync_looper(
        IntTuple* mapData0,
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
                   int max_weight)
    {
#pragma HLS INTERFACE m_axi port=scanPoints0  offset=slave bundle=scan0mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=scanPoints1  offset=slave bundle=scan1mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=mapData0     offset=slave bundle=map0mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=mapData1     offset=slave bundle=map1mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries0 offset=slave bundle=entry0mem latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries1 offset=slave bundle=entry1mem latency=22 depth=18491

        int step = numPoints / SPLIT_FACTOR + 1;
        int last_step = numPoints - (SPLIT_FACTOR - 1) * step;
        tsdf_dataflower(scanPoints0,
                        scanPoints1,
                        step,
                        last_step,
                        new_entries0,
                        new_entries1,
                        sizeX,   sizeY,   sizeZ,
                        posX,    posY,    posZ,
                        offsetX, offsetY, offsetZ,
                        tau);


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
