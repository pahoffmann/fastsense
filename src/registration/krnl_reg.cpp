/**
 * @author Marcel Flottmann
 */

#include <map/local_map_hw.h>

struct IntTuple
{
    int first;
    int second;
};

extern "C"
{

    void krnl_local_map_test(IntTuple* mapData,
                             int sizeX,
                             int sizeY,
                             int sizeZ,
                             int posX,
                             int posY,
                             int posZ,
                             int offsetX,
                             int offsetY,
                             int offsetZ,
                             int* pointData,
                             int numPoints
                            )
    {
#pragma HLS DATA_PACK variable=mapData
#pragma HLS INTERFACE m_axi port=mapData offset=slave bundle=gmem
#pragma HLS INTERFACE m_axi port=pointData offset=slave bundle=gmem
#pragma HLS INTERFACE s_axilite port=mapData bundle=control
#pragma HLS INTERFACE s_axilite port=pointData bundle=control
#pragma HLS INTERFACE s_axilite port=sizeX bundle=control
#pragma HLS INTERFACE s_axilite port=sizeY bundle=control
#pragma HLS INTERFACE s_axilite port=sizeZ bundle=control
#pragma HLS INTERFACE s_axilite port=posX bundle=control
#pragma HLS INTERFACE s_axilite port=posY bundle=control
#pragma HLS INTERFACE s_axilite port=posZ bundle=control
#pragma HLS INTERFACE s_axilite port=offsetX bundle=control
#pragma HLS INTERFACE s_axilite port=offsetY bundle=control
#pragma HLS INTERFACE s_axilite port=offsetZ bundle=control
#pragma HLS INTERFACE s_axilite port=numPoints bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

        fastsense::map::LocalMapHW map{sizeX, sizeY, sizeZ,
                                       posX, posY, posZ,
                                       offsetX, offsetY, offsetZ};
        for (int i = 0; i < numPoints; i += 3 * sizeof(int))
        {
                //todo: do stuff with tsdf and point
                //todo: matrix multiplication???
                //todo: 
                IntTuple tmp = map.get(mapData, pointData[i], pointData[i + sizeof(int)], pointData[i + 2* sizeof(int)]);

        }
        

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
//     }

}
