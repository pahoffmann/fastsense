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
    void test_read_points(PointHW* scanPoints,
                     int numPoints,
                     int tau,
                     hls::stream<PointHW>& point_fifo,
                     hls::stream<int>& distance_fifo,
                     hls::stream<int>& distance_tau_fifo)
    {
    points_loop:
        for (int i = 0; i < numPoints; i++)
        {
#pragma HLS loop_tripcount max=NUM_POINTS
            PointHW p = scanPoints[i];
            int dist = p.norm2();
            int dist_tau = dist + 2 * hls_sqrt_approx(dist) * tau + tau * tau; // (distance + tau)^2
            point_fifo << p;
            distance_fifo << dist;
            distance_tau_fifo << dist_tau;
        }
    }

    void test_update_tsdf(int numPoints,
                     int sizeX,   int sizeY,   int sizeZ,
                     int posX,    int posY,    int posZ,
                     int offsetX, int offsetY, int offsetZ,
                     IntTuple* new_entries,
                     int tau,
                     hls::stream<PointHW>& point_fifo,
                     hls::stream<int>& distance_fifo,
                     hls::stream<int>& distance_tau_fifo)
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
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR*128*2

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
                            current_cell.z += increment.z;
                            err_2 -= direction2.y;
                        }
                        err_1 += direction2.x;
                        err_2 += direction2.z;
                        current_cell.y += increment.y;
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
                    // FIXME: current_distance is (dist)^2, but delta_z needs dist. sqrt is too slow here
                    // TODO: the current fix is to approximate the distance as Moore distance
                    int delta_z = (dz_per_distance * hls_sqrt_approx(current_distance)) / MATRIX_RESOLUTION;
                    interpolate_z = (current_cell.z * MAP_RESOLUTION - delta_z) / MAP_RESOLUTION;
                    interpolate_z_end = (current_cell.z * MAP_RESOLUTION + delta_z) / MAP_RESOLUTION;
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

    void test_tsdf_dataflow(PointHW* scanPoints,
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
#pragma HLS stream variable=point_fifo depth=16
#pragma HLS stream variable=distance_fifo depth=16
#pragma HLS stream variable=distance_tau_fifo depth=16

        test_read_points(scanPoints, numPoints, tau, point_fifo, distance_fifo, distance_tau_fifo);
        test_update_tsdf(numPoints,
                    sizeX,   sizeY,   sizeZ,
                    posX,    posY,    posZ,
                    offsetX, offsetY, offsetZ,
                    new_entries, tau,
                    point_fifo, distance_fifo, distance_tau_fifo);
    }

    void test_sync_loop(
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

    void test_tsdf_dataflower(
        PointHW* scanPoints0,
        PointHW* scanPoints1,
        int step,
        int last_step,
        IntTuple* new_entries0,
        IntTuple* new_entries1,
        int sizeX,   int sizeY,   int sizeZ,
        int posX,    int posY,    int posZ,
        int offsetX, int offsetY, int offsetZ,
        int tau,
        int max_weight)
    {
#pragma HLS dataflow

        test_tsdf_dataflow(scanPoints0 + 0 * step, step,
                      sizeX,   sizeY,   sizeZ,
                      posX,    posY,    posZ,
                      offsetX, offsetY, offsetZ,
                      new_entries0, tau);

        test_tsdf_dataflow(scanPoints1 + 1 * step, last_step,
                      sizeX,   sizeY,   sizeZ,
                      posX,    posY,    posZ,
                      offsetX, offsetY, offsetZ,
                      new_entries1, tau);
    }

    void test_sync_looper(
        IntTuple* mapData0,
        IntTuple* mapData1,
        int step,
        int numPoints,
        IntTuple* new_entries0,
        IntTuple* new_entries1,
        int max_weight)
    {
#pragma HLS dataflow
        test_sync_loop(mapData0, step * 0, step * (0 + 1), new_entries0, max_weight);
        test_sync_loop(mapData1, step * 1, numPoints, new_entries1, max_weight);
    }

    void test_krnl_tsdf(PointHW* scanPoints0,
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
#pragma HLS INTERFACE m_axi port=scanPoints0  offset=slave bundle=scan0mem  latency=22 depth=5760
#pragma HLS INTERFACE m_axi port=scanPoints1  offset=slave bundle=scan1mem  latency=22 depth=5760
#pragma HLS INTERFACE m_axi port=mapData0     offset=slave bundle=map0mem   latency=22 depth=17640
#pragma HLS INTERFACE m_axi port=mapData1     offset=slave bundle=map1mem   latency=22 depth=17640
#pragma HLS INTERFACE m_axi port=new_entries0 offset=slave bundle=entry0mem latency=22 depth=17640
#pragma HLS INTERFACE m_axi port=new_entries1 offset=slave bundle=entry1mem latency=22 depth=17640

        int step = numPoints / SPLIT_FACTOR + 1;
        int last_step = numPoints - (SPLIT_FACTOR - 1) * step;
        test_tsdf_dataflower(scanPoints0,
                        scanPoints1,
                        step,
                        last_step,
                        new_entries0,
                        new_entries1,
                        sizeX,   sizeY,   sizeZ,
                        posX,    posY,    posZ,
                        offsetX, offsetY, offsetZ,
                        tau, max_weight);


        int total_size = sizeX * sizeY * sizeZ;
        int sync_step = total_size / SPLIT_FACTOR + 1;

        test_sync_looper(mapData0,
                    mapData1,
                    sync_step,
                    total_size,
                    new_entries0,
                    new_entries1,
                    max_weight);
    }
}


extern "C"
{
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
                   int max_weight);
}

int main()
{
    PointHW* points = new PointHW[360];
    for (int i = 0; i < 360; i++)
    {
        float r = 1000;
        points[i].x = r * sin(i * M_PI / 180.);
        points[i].y = r * cos(i * M_PI / 180.);
        points[i].z = 0;
    }

    IntTuple* mapData = new IntTuple[21 * 21 * 5];
    IntTuple* newEntries = new IntTuple[21 * 21 * 5];

    IntTuple* mapData_test = new IntTuple[21 * 21 * 5];
    IntTuple* newEntries_test = new IntTuple[21 * 21 * 5];

    for (int i = 0; i < 21 * 21 * 5; i++)
    {
        mapData[i] = std::make_pair(300 / 64, 0);
        newEntries[i] = std::make_pair(300 / 64, 0);
        mapData_test[i] = std::make_pair(300 / 64, 0);
        newEntries_test[i] = std::make_pair(300 / 64, 0);
    }

    std::cout << "Blub\n";
    krnl_tsdf(points, points, 360, mapData, mapData, 21, 21, 5, 0, 0, 0, 10, 10, 10, newEntries, newEntries, 300, 10);
    test_krnl_tsdf(points, points, 360, mapData_test, mapData_test, 21, 21, 5, 0, 0, 0, 10, 10, 10, newEntries_test, newEntries_test, 300, 10);

    int fail_map_cnt = 0;
    int fail_entries_cnt = 0;
    int fail_range_cnt = 0;
    int fail_range_test_cnt = 0;
    for (int i = 0; i < 21 * 21 * 5; i++)
    {
        if (mapData[i] != mapData_test[i])
        {
            std::cout << "Wrong map data " << mapData[i].first << "/" << mapData[i].second << " != " << mapData_test[i].first << "/" << mapData_test[i].second << " at " << i << "\n";
            fail_map_cnt++;
        }
        if (mapData[i].first > 300 || mapData[i].first < -300)
        {
            std::cout << "Incorrect range data " << mapData[i].first << " at " << i << "\n";
            fail_range_cnt++;
        }
        if (mapData_test[i].first > 300 || mapData_test[i].first < -300)
        {
            std::cout << "Incorrect range test data " << mapData[i].first << " at " << i << "\n";
            fail_range_test_cnt++;
        }
        if (newEntries[i] != newEntries_test[i])
        {
            std::cout << "Wrong entry data " << " at " << i << "\n";
            fail_entries_cnt++;
        }
    }

    std::cout << "Map: " << fail_map_cnt << " Entries: " << fail_entries_cnt << " Range: " << fail_range_cnt << " Range test: " << fail_range_test_cnt << "\n";
    return 0;
}








