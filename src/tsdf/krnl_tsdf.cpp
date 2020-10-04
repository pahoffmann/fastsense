/**
 * @author Marc Eisoldt
 */

#include <tsdf/tsdf_hw.h>
#include <map/local_map_hw.h>

#include <cmath>

//#include <util/types.h>

constexpr int MAP_SHIFT = 6; 							// bitshift for a faster way to apply MAP_RESOLUTION
constexpr int MAP_RESOLUTION = 1 << MAP_SHIFT; 			// Resolution of the Map in Millimeter per Cell

constexpr int WEIGHT_SHIFT = 6; 						// bitshift for a faster way to apply WEIGHT_RESOLUTION
constexpr int WEIGHT_RESOLUTION = 1 << WEIGHT_SHIFT; 	// Resolution of the Weights. A weight of 1.0f is represented as WEIGHT_RESOLUTION

constexpr int MATRIX_SHIFT = 10; 						// bitshift for a faster way to apply MATRIX_RESOLUTION
constexpr int MATRIX_RESOLUTION = 1 << MATRIX_SHIFT; 	// Resolutions of calculations with Matrixes

constexpr int RINGS = 16; // TODO: take from Scanner
const int dz_per_distance = std::tan(30.0 / ((double)RINGS - 1.0) / 180.0 * M_PI) / 2.0 * MATRIX_RESOLUTION;

struct Point
{
    int x;
    int y;
    int z;
};

struct IntTuple
{
    int first = 0;
    int second = 0;
};

// constexpr int SCALE = MAP_RESOLUTION;
// constexpr int SIZE_X = 50 * SCALE / MAP_RESOLUTION;
// constexpr int SIZE_Y = 50 * SCALE / MAP_RESOLUTION;
// constexpr int SIZE_Z = 10 * SCALE / MAP_RESOLUTION;

extern "C"
{

    void krnl_tsdf(Point* scanPoints,
                             int numPoints,
                   int*   scannerPos,
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
                             int tau,
                             int max_weight
                            )
    {
#pragma HLS DATA_PACK variable=scanPoints
#pragma HLS INTERFACE m_axi port=scanPoints offset=slave bundle=gmem
#pragma HLS INTERFACE s_axilite port=scanPoints bundle=control
#pragma HLS INTERFACE s_axilite port=numPoints bundle=control 

#pragma HLS DATA_PACK variable=scannerPos
#pragma HLS INTERFACE m_axi port=scannerPos offset=slave bundle=gmem
#pragma HLS INTERFACE s_axilite port=scannerPos bundle=control

#pragma HLS DATA_PACK variable=mapData
#pragma HLS INTERFACE m_axi port=mapData offset=slave bundle=gmem
#pragma HLS INTERFACE s_axilite port=mapData bundle=control
#pragma HLS INTERFACE s_axilite port=sizeX bundle=control
#pragma HLS INTERFACE s_axilite port=sizeY bundle=control
#pragma HLS INTERFACE s_axilite port=sizeZ bundle=control
#pragma HLS INTERFACE s_axilite port=posX bundle=control
#pragma HLS INTERFACE s_axilite port=posY bundle=control
#pragma HLS INTERFACE s_axilite port=posZ bundle=control
#pragma HLS INTERFACE s_axilite port=offsetX bundle=control
#pragma HLS INTERFACE s_axilite port=offsetY bundle=control
#pragma HLS INTERFACE s_axilite port=offsetZ bundle=control

#pragma HLS INTERFACE s_axilite port=tau bundle=control
#pragma HLS INTERFACE s_axilite port=max_weight bundle=control

#pragma HLS INTERFACE s_axilite port=return bundle=control

        fastsense::map::LocalMapHW map{sizeX, sizeY, sizeZ,
                                       posX, posY, posZ,
                                       offsetX, offsetY, offsetZ};

        int weight_epsilon = tau / 10;
        IntTuple new_entries[sizeX * sizeY * sizeZ];

        for(int point_index = 0; point_index < numPoints; ++point_index)
        {
            int direction[3];

            direction[0] = scanPoints[point_index].x - scannerPos[0];
            direction[1] = scanPoints[point_index].y - scannerPos[1];
            direction[2] = scanPoints[point_index].z - scannerPos[2];

            int distance = norm(direction);

            int prev[] = {0, 0, 0};

            for(int len = MAP_RESOLUTION; len <= distance + tau; len += MAP_RESOLUTION)
            {
                int proj[3];
                int index[3];

                for(int coor_index = 0; coor_index < 3; ++coor_index)
                {
                    proj[coor_index] = scannerPos[coor_index] + direction[coor_index] * len / distance;
                    index[coor_index] = proj[coor_index] / MAP_RESOLUTION;
                }

                if(index[0] == prev[0] && index[1] == prev[1])
                {
                    continue;
                }

                for(int coor_index = 0; coor_index < 3; ++coor_index)
                {
                    prev[coor_index] = index[coor_index];
                }

                if(!map.inBounds(index[0], index[1], index[2]))
                {
                    continue;
                }

                int target_center[3];
                int value_vec[3];

                for(int coor_index = 0; coor_index < 3; ++coor_index)
                {
                    target_center[coor_index] = index[coor_index] * MAP_RESOLUTION + MAP_RESOLUTION / 2;
                }            

                value_vec[0] = scanPoints[point_index].x - target_center[0];
                value_vec[1] = scanPoints[point_index].y - target_center[1];
                value_vec[2] = scanPoints[point_index].z - target_center[2];

                int value = norm(value_vec);

                if(tau < value)
                {
                    value = tau;
                }

                if(len > distance)
                {
                    value = -value;
                }

                int weight = WEIGHT_RESOLUTION;

                if(value < - weight_epsilon)
                {
                    weight = WEIGHT_RESOLUTION * (tau + value) / (tau - weight_epsilon);
                }

                if(weight == 0)
                {
                    continue;
                }

                //for(inde)

                int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;
                int lowest = (proj[2] - delta_z) / MAP_RESOLUTION;
                int highest = (proj[2] + delta_z) / MAP_RESOLUTION;

                for(index[2] = lowest; index[2] <= highest; ++index[2])
                {
                    auto entry = map.get(new_entries, index[0], index[1], index[2]);
            
                    if(!map.inBounds(index[0], index[1], index[2]))
                    {
                        continue;
                    }

                    if(entry.second == 0 || std::abs(value) < std::abs(entry.first)) 
                    {
                        entry.first = value;
                        entry.second = weight;
                    }

                    map.set(new_entries, index[0], index[1], index[2], entry);
                    }
            }
        }

        for (int i = 0; i < map.sizeX; i++)
        {
            for (int j = 0; j < map.sizeY; j++)
            {
                for (int k = 0; k < map.sizeZ; k++)
                {
                    int index = i + j * sizeX + k * sizeX * sizeY;
                    int new_weight = mapData[index].second + new_entries[index].second;

                    if(new_weight > max_weight)
                    {
                        new_weight = max_weight;
                    }

                    mapData[index].first = (mapData[index].first * mapData[index].second + new_entries[index].first * new_entries[index].second) / new_weight;
                    mapData[index].second = new_weight; 
                }
            }
        }

        // Map access example 

//         for (int i = map.posX - map.sizeX / 2; i <= map.posX + map.sizeX / 2; i++)
//         {
//             for (int j = map.posY - map.sizeY / 2; j <= map.posY + map.sizeY / 2; j++)
//             {
//                 for (int k = map.posZ - map.sizeZ / 2; k <= map.posZ + map.sizeZ / 2; k++)
//                 {
// #pragma HLS PIPELINE
//                     IntTuple tmp = map.get(mapData, i, j, k);
//                     tmp.first *= 2;
//                     tmp.second /= 2;
//                     map.set(mapData, i, j, k, tmp);
//                 }
//             }
//         }

        // Map and point combination example

        // for(int i = 0; i < numPoints; ++i)
        // {
        //     Point& point = scanPoints[i];
        //     IntTuple tmp = map.get(mapData, point.x, point.y, point.z);
        //     tmp.first = 42;
        //     tmp.second = 17;
        //     map.set(mapData, point.x, point.y, point.z, tmp);
        // }
    }

}