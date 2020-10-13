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

//constexpr unsigned int SCALE = 1000;

//constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
//constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
//constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION;

//constexpr int MAX_DISTANCE = (SIZE_X + SIZE_Y + SIZE_Z) / 2 * MAP_RESOLUTION;//(int)(std::sqrt(SIZE_X * SIZE_X + SIZE_Y * SIZE_Y + SIZE_Z * SIZE_Z) / 2 * MAP_RESOLUTION);
//constexpr int MAX_TSDF_IT = MAX_DISTANCE / (MAP_RESOLUTION / 2);

//constexpr int MAX_INTERPOLATE_IT = 2 * dz_per_distance * MAX_DISTANCE / MATRIX_RESOLUTION / MAP_RESOLUTION;

constexpr int NUM_POINTS = 30000;

using IntTuple = std::pair<int, int>;

extern "C"
{
    void read_points(PointHW* scanPoints,
                     int numPoints,
                     int tau,
                     hls::stream<PointHW>& point_fifo,
                     hls::stream<int>& distance_fifo,
                     hls::stream<int>& distance_tau_fifo)
    {
    points_loop:
        for (int i = 0; i < numPoints; i++)
        {
#pragma HLS loop_tripcount max=30000
            PointHW p = scanPoints[i];
            int dist = p.norm2();
            int dist_tau = dist + 2 * hls_sqrt_approx(dist) * tau + tau * tau; // (distance + tau)^2
            point_fifo << p;
            distance_fifo << dist;
            distance_tau_fifo << dist_tau;
        }
    }

    void update_tsdf(int numPoints,
                     int sizeX,
                     int sizeY,
                     int sizeZ,
                     int posX,
                     int posY,
                     int posZ,
                     int offsetX,
                     int offsetY,
                     int offsetZ,
                     IntTuple* new_entries,
                     int tau,
                     hls::stream<PointHW>& point_fifo,
                     hls::stream<int>& distance_fifo,
                     hls::stream<int>& distance_tau_fifo)
    {
        fastsense::map::LocalMapHW map{sizeX, sizeY, sizeZ,
                                       posX, posY, posZ,
                                       offsetX, offsetY, offsetZ};
        PointHW map_pos{posX, posY, posZ};
        PointHW map_offset{offsetX, offsetY, offsetZ};

        int weight_epsilon = tau / 10;

        // squared distances
        int distance;
        int distance_tau;
        int current_distance;

        PointHW current_point;
        PointHW current_cell;
        PointHW direction;
        PointHW direction2;
        PointHW increment;
        PointHW abs_direction;

        int err_1;
        int err_2;

        int interpolate_z;
        int interpolate_z_end;

        int point_idx = 0;

        bool load_new_point = true;
    tsdf_loop:
        while (point_idx < numPoints)
        {
#pragma HLS loop_tripcount max=30000*128*2

            // Load point and init Bresenham
            if (load_new_point)
            {
                point_fifo >> current_point;
                distance_fifo >> distance;
                distance_tau_fifo >> distance_tau;

                current_distance = 0;
                current_cell = map_pos;

                direction = current_point.to_map() - map_pos;
                increment = direction.sign();
                abs_direction = direction.abs();
                direction2 = abs_direction * 2;

                if ((abs_direction.x >= abs_direction.y) && (abs_direction.x >= abs_direction.z))
                {
                    err_1 = direction2.y - abs_direction.x;
                    err_2 = direction2.z - abs_direction.x;
                }
                else if ((abs_direction.y >= abs_direction.x) && (abs_direction.y >= abs_direction.z))
                {
                    err_1 = direction2.x - abs_direction.y;
                    err_2 = direction2.z - abs_direction.y;
                }
                else
                {
                    err_1 = direction2.y - abs_direction.z;
                    err_2 = direction2.x - abs_direction.z;
                }

                interpolate_z = 0;
                interpolate_z_end = 0;

                load_new_point = false;
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
#pragma HLS dependence variable=new_entries inter false
                IntTuple entry = map.get(new_entries, current_cell.x, current_cell.y, interpolate_z);
                IntTuple new_entry;
                
                if (entry.second == 0 || hls_abs(tsdf_value) < hls_abs(entry.first))
                {
                    new_entry.first = tsdf_value;
                    new_entry.second = weight;
                }
                else
                {
                    new_entry.first = entry.first;
                    new_entry.second = entry.second;
                }

                map.set(new_entries, current_cell.x, current_cell.y, interpolate_z, new_entry);
            }

            // Update current_cell with Bresenham and interpolation
            if (interpolate_z == interpolate_z_end)
            {
                if (current_distance < distance_tau)
                {
                    int approx_distance;
                    if ((abs_direction.x >= abs_direction.y) && (abs_direction.x >= abs_direction.z))
                    {
                        if (err_1 > 0)
                        {
                            current_cell.y += increment.y;
                            err_1 -= direction2.x;
                        }
                        if (err_2 > 0)
                        {
                            current_cell.z += increment.z;
                            err_2 -= direction2.x;
                        }
                        err_1 += direction2.y;
                        err_2 += direction2.z;
                        current_cell.x += increment.x;

                        approx_distance = current_cell.x;
                    }
                    else if ((abs_direction.y >= abs_direction.x) && (abs_direction.y >= abs_direction.z))
                    {
                        if (err_1 > 0)
                        {
                            current_cell.x += increment.x;
                            err_1 -= direction2.y;
                        }
                        if (err_2 > 0)
                        {
                            current_cell.y += increment.z;
                            err_2 -= direction2.y;
                        }
                        err_1 += direction2.x;
                        err_2 += direction2.z;
                        current_cell.z += increment.y;

                        approx_distance = current_cell.y;
                    }
                    else
                    {
                        if (err_1 > 0)
                        {
                            current_cell.y += increment.y;
                            err_1 -= direction2.z;
                        }
                        if (err_2 > 0)
                        {
                            current_cell.x += increment.x;
                            err_2 -= direction2.z;
                        }
                        err_1 += direction2.y;
                        err_2 += direction2.x;
                        current_cell.z += increment.z;

                        approx_distance = current_cell.z;
                    }

                    current_distance = (current_cell - map_pos).to_mm().norm2();
                    // FIXME: current_distance is (dist)^2, but delta_z needs dist. sqrt is too slow here
                    // TODO: the current fix is to approximate the distance as Moore distance
                    int delta_z = (dz_per_distance * hls_sqrt_approx(current_distance)) / MATRIX_RESOLUTION / MAP_RESOLUTION;
                    interpolate_z = current_cell.z - delta_z;
                    interpolate_z_end = current_cell.z + delta_z;
                }
                else
                {
                    point_idx++;
                    load_new_point = true;
                }
            }
            else
            {
                interpolate_z++;
            }
        }
    }

    void tsdf_dataflow(PointHW* scanPoints,
                       int numPoints,
                       int sizeX,
                       int sizeY,
                       int sizeZ,
                       int posX,
                       int posY,
                       int posZ,
                       int offsetX,
                       int offsetY,
                       int offsetZ,
                       IntTuple* new_entries,
                       int tau)
    {
#pragma HLS dataflow
        hls::stream<PointHW> point_fifo;
        hls::stream<int> distance_fifo;
        hls::stream<int> distance_tau_fifo;
#pragma HLS stream variable=point_fifo depth=16
#pragma HLS stream variable=distance_fifo depth=16
#pragma HLS stream variable=distance_tau_fifo depth=16

        read_points(scanPoints, numPoints, tau, point_fifo, distance_fifo, distance_tau_fifo);
        update_tsdf(numPoints,
                    sizeX, sizeY, sizeZ,
                    posX, posY, posZ,
                    offsetX, offsetY, offsetZ,
                    new_entries, tau,
                    point_fifo, distance_fifo, distance_tau_fifo);
    }

    void krnl_tsdf(PointHW* scanPoints,
                   int numPoints,
                   IntTuple* mapData,
                   int sizeX,
                   int sizeY,
                   int sizeZ,
                   int posX,
                   int posY,
                   int posZ,
                   int offsetX,
                   int offsetY,
                   int offsetZ,
                   IntTuple* new_entries,
                   int tau,
                   int max_weight)
    {
#pragma HLS INTERFACE m_axi port=scanPoints offset=slave bundle=scanmem latency=8
#pragma HLS INTERFACE m_axi port=mapData offset=slave bundle=mapmem  latency=8
#pragma HLS INTERFACE m_axi port=new_entries offset=slave bundle=entrymem  latency=8

        fastsense::map::LocalMapHW map{sizeX, sizeY, sizeZ,
                                       posX, posY, posZ,
                                       offsetX, offsetY, offsetZ};

        tsdf_dataflow(scanPoints, numPoints,
                      sizeX, sizeY, sizeZ,
                      posX, posY, posZ,
                      offsetX, offsetY, offsetZ,
                      new_entries, tau);

    sync_loop:
        for (int index = 0; index < sizeX * sizeY * sizeZ; index++)
        {
#pragma HLS loop_tripcount min=8000000 max=8000000
#pragma HLS pipeline II=1
#pragma HLS dependence variable=mapData inter false
#pragma HLS dependence variable=new_entries inter false

            IntTuple map_entry = mapData[index];
            IntTuple new_entry = new_entries[index];

            int new_weight = map_entry.second + new_entry.second;

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
