/**
 * @author Marc Eisoldt
 */

#include <tsdf/kernel/tsdf_hw.h>
#include <map/local_map_hw.h>

#include <iostream>

#include <ap_fixed.h>
#include <hls_stream.h>

//#include <util/types.h>

using namespace fastsense::map;

constexpr int MAP_SHIFT = 6;                            // bitshift for a faster way to apply MAP_RESOLUTION
constexpr int MAP_RESOLUTION = 1 << MAP_SHIFT;          // Resolution of the Map in Millimeter per Cell

constexpr int WEIGHT_SHIFT = 6;                         // bitshift for a faster way to apply WEIGHT_RESOLUTION
constexpr int WEIGHT_RESOLUTION = 1 << WEIGHT_SHIFT;    // Resolution of the Weights. A weight of 1.0f is represented as WEIGHT_RESOLUTION

constexpr int MATRIX_SHIFT = 10;                        // bitshift for a faster way to apply MATRIX_RESOLUTION
constexpr int MATRIX_RESOLUTION = 1 << MATRIX_SHIFT;    // Resolutions of calculations with Matrixes

constexpr int RINGS = 16; // TODO: take from Scanner
const int dz_per_distance = (int)(std::tan(30.0 / ((double)RINGS - 1.0) / 180.0 * M_PI) / 2.0 * MATRIX_RESOLUTION);

//constexpr unsigned int SCALE = 1000;

//constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
//constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
//constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION;

//constexpr int MAX_DISTANCE = (SIZE_X + SIZE_Y + SIZE_Z) / 2 * MAP_RESOLUTION;//(int)(std::sqrt(SIZE_X * SIZE_X + SIZE_Y * SIZE_Y + SIZE_Z * SIZE_Z) / 2 * MAP_RESOLUTION);
//constexpr int MAX_TSDF_IT = MAX_DISTANCE / (MAP_RESOLUTION / 2);

//constexpr int MAX_INTERPOLATE_IT = 2 * dz_per_distance * MAX_DISTANCE / MATRIX_RESOLUTION / MAP_RESOLUTION;

constexpr int NUM_POINTS = 30000;

struct Point
{
    int x;
    int y;
    int z;
    int dummy; //128 bit padding

    Point operator-(const Point& rhs)
    {
        Point p;
        p.x = x - rhs.x;
        p.y = y - rhs.y;
        p.z = z - rhs.z;
        return p;
    }

    Point operator*(int rhs)
    {
        Point p;
        p.x = x * rhs;
        p.y = y * rhs;
        p.z = z * rhs;
        return p;
    }

    Point& operator=(const Point& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }

    Point& operator=(int rhs)
    {
        x = rhs;
        y = rhs;
        z = rhs;
        return *this;
    }

    int norm2()
    {
        return x * x + y * y + z * z;
    }

    int norm()
    {
        return hls::sqrt(norm2());
    }

    Point abs()
    {
        Point p;
        p.x = hls::abs(x);
        p.y = hls::abs(y);
        p.z = hls::abs(z);
        return p;
    }

    Point sign()
    {
        Point p;
        p.x = x < 0 ? -1 : 1;
        p.y = y < 0 ? -1 : 1;
        p.z = z < 0 ? -1 : 1;
        return p;
    }

    Point to_map()
    {
        Point p;
        p.x = x >> MAP_SHIFT;
        p.y = y >> MAP_SHIFT;
        p.z = z >> MAP_SHIFT;
        return p;
    }

    Point to_mm()
    {
        Point p;
        p.x = (x << MAP_SHIFT) + MAP_RESOLUTION / 2;
        p.y = (y << MAP_SHIFT) + MAP_RESOLUTION / 2;
        p.z = (z << MAP_SHIFT) + MAP_RESOLUTION / 2;
        return p;
    }
};

struct IntTuple
{
    int first = 0;
    int second = 0;
};

extern "C"
{
    void read_points(Point* scanPoints,
                     int numPoints,
                     int tau,
                     hls::stream<Point>& point_fifo,
                     hls::stream<int>& distance_fifo,
                     hls::stream<int>& distance_tau_fifo)
    {
    points_loop:
        for (int i = 0; i < numPoints; i++)
        {
#pragma HLS loop_tripcount max=30000
            Point p = scanPoints[i];
            int dist = p.norm2();
            int dist_tau = dist + 2 * hls::sqrt(dist) * tau + tau * tau; // (distance + tau)^2
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
                     ap_fixed<32, 2> tau_inverse,
                     int weight_epsilon,
                     hls::stream<Point>& point_fifo,
                     hls::stream<int>& distance_fifo,
                     hls::stream<int>& distance_tau_fifo)
    {
        fastsense::map::LocalMapHW map{sizeX, sizeY, sizeZ,
                                       posX, posY, posZ,
                                       offsetX, offsetY, offsetZ};
        Point map_pos{posX, posY, posZ};
        Point map_offset{offsetX, offsetY, offsetZ};

        // squared distances
        int distance;
        int distance_tau;
        int current_distance;

        Point current_point;
        Point current_cell;
        Point cell_center;
        Point direction;
        Point direction2;
        Point increment;
        Point abs_direction;

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
                weight = WEIGHT_RESOLUTION * (tau + tsdf_value) * tau_inverse;
            }

            if (weight != 0 && map.in_bounds(current_cell.x, current_cell.y, interpolate_z))
            {
#pragma HLS dependence variable=new_entries inter false
                IntTuple entry = map.get(new_entries, current_cell.x, current_cell.y, interpolate_z);
                IntTuple new_entry;

                if (entry.second == 0 || hls::abs(tsdf_value) < hls::abs(entry.first))
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
                    }

                    current_distance = (current_cell - map_pos).to_mm().norm2();
                    int delta_z = (dz_per_distance * current_distance) >> MATRIX_SHIFT;
                    interpolate_z = (current_cell.z - delta_z) >> MAP_SHIFT;
                    interpolate_z_end = (current_cell.z + delta_z) >> MAP_SHIFT;
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

    void tsdf_dataflow(Point* scanPoints,
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
                       int tau,
                       ap_fixed<32, 2> tau_inverse,
                       int weight_epsilon)
    {
#pragma HLS dataflow
        hls::stream<Point> point_fifo;
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
                    new_entries, tau, tau_inverse, weight_epsilon,
                    point_fifo, distance_fifo, distance_tau_fifo);
    }

    void krnl_tsdf(Point* scanPoints,
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

        int weight_epsilon = tau / 10;
        ap_fixed<32, 2> tau_inverse = 1.f / (tau - weight_epsilon);

        tsdf_dataflow(scanPoints, numPoints,
                      sizeX, sizeY, sizeZ,
                      posX, posY, posZ,
                      offsetX, offsetY, offsetZ,
                      new_entries, tau, tau_inverse, weight_epsilon);

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
