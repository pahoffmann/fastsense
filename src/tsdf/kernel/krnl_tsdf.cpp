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
                     int sizeX,   int sizeY,   int sizeZ,
                     int posX,    int posY,    int posZ,
                     int offsetX, int offsetY, int offsetZ,
                     int tau,
                     hls::stream<int>& value_fifo,
                     hls::stream<int>& weight_fifo,
                     hls::stream<PointHW>& index_fifo,
                     hls::stream<int>& lowest_fifo,
                     hls::stream<int>& highest_fifo
                     )
    {
        fastsense::map::LocalMapHW map{sizeX,   sizeY,   sizeZ,
                                       posX,    posY,    posZ,
                                       offsetX, offsetY, offsetZ};
        PointHW map_pos{posX, posY, posZ};

        int weight_epsilon = tau / 10;

    points_loop:
        for (int point_idx = 0; point_idx < numPoints; point_idx++)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR

            PointHW scan_point = scanPoints[point_idx];

            PointHW direction = scan_point - map_pos.to_mm();

            int distance = direction.norm();
            int distance_tau = distance + tau;
            int inv_distance = MATRIX_RESOLUTION / distance;

            int prev[2] = {0, 0};
#pragma HLS ARRAY_PARTITION variable=prev complete

        tsdf_loop:
            for (int len = MAP_RESOLUTION; len <= distance_tau; len += MAP_RESOLUTION)
            {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount min=0 max=128

                PointHW proj = map_pos.to_mm() + direction * len * inv_distance / MATRIX_RESOLUTION;
                PointHW index = proj.to_map();

                if (index.x == prev[0] && index.y == prev[1])
                {
                    continue;
                }
                prev[0] = index.x;
                prev[1] = index.y;

                if (!map.in_bounds(index.x, index.y, index.z))
                {
                    continue;
                }

                int tsdf_value = (scan_point - index.to_mm()).norm();
                if (tsdf_value > tau)
                {
                    tsdf_value = tau;
                }

                if (len > distance)
                {
                    tsdf_value = -tsdf_value;
                }

                int weight = WEIGHT_RESOLUTION;

                if (tsdf_value < -weight_epsilon)
                {
                    weight = WEIGHT_RESOLUTION * (tau + tsdf_value) / (tau - weight_epsilon);
                }

                if (weight == 0)
                {
                    continue;
                }

                int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;
                int lowest = (proj.z - delta_z) / MAP_RESOLUTION;
                int highest = (proj.z + delta_z) / MAP_RESOLUTION;
                if (lowest < posZ - sizeZ / 2)
                {
                    lowest = posZ - sizeZ / 2;
                }
                if (highest < posZ + sizeZ / 2)
                {
                    highest = posZ + sizeZ / 2;
                }

                value_fifo << tsdf_value;
                weight_fifo << weight;
                index_fifo << index;
                lowest_fifo << lowest;
                highest_fifo << highest;
            }
        }
    }

    void update_tsdf(int numPoints,
                     int sizeX,   int sizeY,   int sizeZ,
                     int posX,    int posY,    int posZ,
                     int offsetX, int offsetY, int offsetZ,
                     IntTuple* new_entries,
                     int tau,
                     hls::stream<int>& value_fifo,
                     hls::stream<int>& weight_fifo,
                     hls::stream<PointHW>& index_fifo,
                     hls::stream<int>& lowest_fifo,
                     hls::stream<int>& highest_fifo)
    {
        fastsense::map::LocalMapHW map{sizeX,   sizeY,   sizeZ,
                                       posX,    posY,    posZ,
                                       offsetX, offsetY, offsetZ};
        PointHW map_pos{posX, posY, posZ};

        int tsdf_value;
        int weight;
        PointHW index;
        int lowest;
        int highest;

    update_loop:
        for (int point_idx = 0; point_idx < numPoints; ++point_idx)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/SPLIT_FACTOR

        tsdf_loop:
			for (int len = MAP_RESOLUTION; len <= MAP_RESOLUTION + 128; len += MAP_RESOLUTION)
			{
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount min=0 max=128

				value_fifo >> tsdf_value;
				weight_fifo >> weight;
				index_fifo >> index;
				lowest_fifo >> lowest;
				highest_fifo >> highest;

			interpolate_loop:
				for (int z = lowest; z <= highest; ++z)
				{
	#pragma HLS loop_tripcount min=1 max=2
	#pragma HLS pipeline II=1
	#pragma HLS dependence variable=new_entries inter false

					if (!map.in_bounds(index.x, index.y, z))
					{
						continue;
					}

					IntTuple entry = map.get(new_entries, index.x, index.y, z);

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

					map.set(new_entries, index.x, index.y, z, new_entry);
				}
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
        hls::stream<int> value_fifo;
        hls::stream<int> weight_fifo;
        hls::stream<PointHW> index_fifo;
        hls::stream<int> lowest_fifo;
        hls::stream<int> highest_fifo;
#pragma HLS stream depth=16 variable=value_fifo
#pragma HLS stream depth=16 variable=weight_fifo
#pragma HLS stream depth=16 variable=index_fifo
#pragma HLS stream depth=16 variable=lowest_fifo
#pragma HLS stream depth=16 variable=highest_fifo

        read_points(scanPoints, numPoints,
                    sizeX,   sizeY,   sizeZ,
                    posX,    posY,    posZ,
                    offsetX, offsetY, offsetZ,
                    tau,
                    value_fifo,
                    weight_fifo,
                    index_fifo,
                    lowest_fifo,
                    highest_fifo);

        update_tsdf(numPoints,
                    sizeX,   sizeY,   sizeZ,
                    posX,    posY,    posZ,
                    offsetX, offsetY, offsetZ,
                    new_entries, tau,
                    value_fifo,
                    weight_fifo,
                    index_fifo,
                    lowest_fifo,
                    highest_fifo);
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
