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

extern "C"
{
    void read_points(PointHW* scanPoints,
                     int numPoints,
                     int tau,
                     PointHW map_pos,
                     hls::stream<PointHW>& point_fifo,
                     hls::stream<int>& distance_fifo,
                     hls::stream<int>& distance_tau_fifo,
                     hls::stream<PointHW>& direction_fifo,
                     hls::stream<PointHW>& abs_direction_fifo,
                     hls::stream<int>& lower_z_fifo,
                     hls::stream<int>& upper_z_fifo)
    {
    points_loop:
        for (int i = 0; i < numPoints; i++)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR
            PointHW p = scanPoints[(i * 157) % numPoints];
            int dist = p.norm();
            PointHW direction = p.to_map() - map_pos;
            PointHW abs_direction = direction.abs();
            int delta_z = dz_per_distance * dist / MATRIX_RESOLUTION;
            int lower_z = (p.z - delta_z) / MAP_RESOLUTION;
            int upper_z = (p.z + delta_z) / MAP_RESOLUTION;
            int distance = abs_direction.x >= abs_direction.y ? abs_direction.x : abs_direction.y;
            int distance_tau = distance + tau / MAP_RESOLUTION;
            point_fifo << p;
            distance_fifo << distance;
            distance_tau_fifo << distance_tau;
            direction_fifo << direction;
            abs_direction_fifo << abs_direction;
            lower_z_fifo << lower_z;
            upper_z_fifo << upper_z;
        }
    }

    void update_tsdf(int numPoints,
                     int sizeX,   int sizeY,   int sizeZ,
                     int posX,    int posY,    int posZ,
                     int offsetX, int offsetY, int offsetZ,
                     IntTuple* new_entries,
                     int tau,
                     hls::stream<PointHW>& point_fifo,
                     hls::stream<int>& distance_fifo,
                     hls::stream<int>& distance_tau_fifo,
                     hls::stream<PointHW>& direction_fifo,
                     hls::stream<PointHW>& abs_direction_fifo,
                     hls::stream<int>& lower_z_fifo,
                     hls::stream<int>& upper_z_fifo)
    {
        fastsense::map::LocalMapHW map{sizeX,   sizeY,   sizeZ,
                                       posX,    posY,    posZ,
                                       offsetX, offsetY, offsetZ};
        PointHW map_pos{posX, posY, posZ};
        PointHW map_offset{offsetX, offsetY, offsetZ};

        int weight_epsilon = tau / 10;

        // squared distances
        int distance;
        int distance_tau;
        int lower_z;
        int upper_z;

        int current_distance;

        PointHW current_point;
        PointHW current_cell;
        PointHW direction;
        PointHW abs_direction;

        PointHW increment;
        int inc_lower_z;
        int inc_upper_z;

        int abs_major;
        int abs_minor;
        int abs2_major;
        int abs2_minor;
        int abs2_lower_z;
        int abs2_upper_z;

        bool is_x_major; // else y major
        int err_minor;
        int err_lower_z;
        int err_upper_z;

        int interpolate_z;
        int interpolate_z_end;

        int point_idx = 0;

        bool load_new_point = true;
    tsdf_loop:
        while (point_idx < numPoints)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR*128*2

            // Load point and init Bresenham
            if (load_new_point)
            {
                point_fifo >> current_point;
                distance_fifo >> distance;
                distance_tau_fifo >> distance_tau;
                direction_fifo >> direction;
                abs_direction_fifo >> abs_direction;
                lower_z_fifo >> lower_z;
                upper_z_fifo >> upper_z;

                current_distance = 0;
                current_cell = map_pos;

                if (abs_direction.x >= abs_direction.y)
                {
                    is_x_major = true;
                    abs_major = abs_direction.x;
                    abs_minor = abs_direction.y;
                }
                else
                {
                    is_x_major = false;
                    abs_major = abs_direction.y;
                    abs_minor = abs_direction.x;
                }

                abs2_major = abs_major * 2;
                abs2_minor = abs_minor * 2;
                err_minor = abs2_minor - abs_major;

                increment = direction.sign();
                inc_lower_z = lower_z < 0 ? -1 : 1;
                inc_upper_z = upper_z < 0 ? -1 : 1;

                abs2_lower_z = hls_abs(lower_z) * 2;
                abs2_upper_z = hls_abs(upper_z) * 2;
                err_lower_z = abs2_lower_z - abs_major;
                err_upper_z = abs2_upper_z - abs_major;

                lower_z = upper_z = current_cell.z;
                interpolate_z = lower_z;
                interpolate_z_end = upper_z;

                load_new_point = false;
            }
            else if (current_distance >= distance_tau)
            {
                // this is in 'else' to convince Vitis that there is at least one iteration per Point and
                // the condition won't be checked in the same iteration in which distance_tau is read from fifo

                point_idx++;
                load_new_point = true;
            }

            // Calculate and update TSDF value
            int tsdf_value = (current_point - current_cell.to_mm()).norm();
            if (tsdf_value > tau)
            {
                tsdf_value = tau;
            }

            if (current_distance > distance)
            {
                tsdf_value = -tsdf_value;
            }

            int weight = WEIGHT_RESOLUTION;

            if (tsdf_value < -weight_epsilon)
            {
                weight = WEIGHT_RESOLUTION * (tau + tsdf_value) / (tau - weight_epsilon);
            }

            if (weight != 0 && map.in_bounds(current_cell.x, current_cell.y, interpolate_z))
            {
                int index = map.getIndex(current_cell.x, current_cell.y, interpolate_z);
#pragma HLS dependence variable=new_entries inter false
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
                if (err_lower_z > 0)
                {
                    lower_z += inc_lower_z;
                    err_lower_z -= abs2_major;
                }
                if (err_upper_z > 0)
                {
                    upper_z += inc_upper_z;
                    err_upper_z -= abs2_major;
                }

                if (err_minor > 0)
                {
                    if (is_x_major)
                    {
                        current_cell.y += increment.y;
                    }
                    else
                    {
                        current_cell.x += increment.x;
                    }
                    err_minor -= abs2_major;
                }
                else
                {
                    if (is_x_major)
                    {
                        current_cell.x += increment.x;
                    }
                    else
                    {
                        current_cell.y += increment.y;
                    }
                    current_distance++;
                    err_minor += abs2_minor;
                    err_lower_z += abs2_lower_z;
                    err_upper_z += abs2_upper_z;
                }

                current_cell.z = (lower_z + upper_z) / 2;
                interpolate_z = lower_z;
                interpolate_z_end = upper_z;
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
        hls::stream<int> distance_fifo;
        hls::stream<int> distance_tau_fifo;
        hls::stream<PointHW> direction_fifo;
        hls::stream<PointHW> abs_direction_fifo;
        hls::stream<int> lower_z_fifo;
        hls::stream<int> upper_z_fifo;
#pragma HLS stream variable=point_fifo depth=16
#pragma HLS stream variable=distance_fifo depth=16
#pragma HLS stream variable=distance_tau_fifo depth=16
#pragma HLS stream variable=direction_fifo depth=16
#pragma HLS stream variable=abs_direction_fifo depth=16
#pragma HLS stream variable=lower_z_fifo depth=16
#pragma HLS stream variable=upper_z_fifo depth=16

        read_points(scanPoints, numPoints, tau,
                    PointHW{posX, posY, posZ},
                    point_fifo,
                    distance_fifo,
                    distance_tau_fifo,
                    direction_fifo,
                    abs_direction_fifo,
                    lower_z_fifo,
                    upper_z_fifo);

        update_tsdf(numPoints,
                    sizeX,   sizeY,   sizeZ,
                    posX,    posY,    posZ,
                    offsetX, offsetY, offsetZ,
                    new_entries, tau,
                    point_fifo,
                    distance_fifo,
                    distance_tau_fifo,
                    direction_fifo,
                    abs_direction_fifo,
                    lower_z_fifo,
                    upper_z_fifo);
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
