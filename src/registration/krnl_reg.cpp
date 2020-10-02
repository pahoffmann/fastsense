/**
 * @author Marcel Flottmann
 */

#include <map/local_map_hw.h>
#include <msg/point.h>
struct IntTuple
{
    int first;
    int second;
};

struct IntPoint
{
    int x;
    int y;
    int z;
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
                             fastsense::msg::Point* pointData,
                             int numPoints,
                             int mapResolution
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
#pragma HLS INTERFACE s_axilite port=mapResolution bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

        fastsense::map::LocalMapHW map{sizeX, sizeY, sizeZ,
                                       posX, posY, posZ,
                                       offsetX, offsetY, offsetZ};


        for (int i = 0; i < numPoints; i += sizeof(IntPoint))
        {
            //todo: do stuff with tsdf and point
            //todo: matrix multiplication???
            //todo: 
            IntTuple tmp = map.get(mapData, pointData[i].x, pointData[i].y, pointData[i].z);

            //#pragma omp for schedule(static) nowait
            
            auto point = pointData[i];

            // apply the total transform
            //Vector3i point = transform(input, next_transform);

            // fastsense::msg::Point buf;    
            // buf.x = point.x / mapResolution;
            // buf.y = point.y / mapResolution;
            // buf.z = point.z / mapResolution;
            //Vector3i buf = floor_shift(point, MAP_SHIFT);
            
            try
            {
                const auto& current = map.get(mapData, pointData[i].x / mapResolution, pointData[i].y / mapResolution, pointData[i].z / mapResolution);

                if (current.second == 0)
                {
                    continue;
                }

                gradient = Vector3i::Zero();
                for (size_t axis = 0; axis < 3; axis++)
                {
                    Vector3i index = buf;
                    index[axis] -= 1;
                    const auto last = localmap.value(index);
                    index[axis] += 2;
                    const auto next = localmap.value(index);
                    if (last.second != 0 && next.second != 0 && (next.first > 0.0) == (last.first > 0.0))
                    {
                        gradient[axis] = (next.first - last.first) / 2;
                    }
                }

                jacobi << point.cross(gradient).cast<long>(), gradient.cast<long>();

                local_h += jacobi * jacobi.transpose();
                local_g += jacobi * current.first;
                local_error += abs(current.first);
                local_count++;
            }
            catch (const std::out_of_range&)
            {

            }
        

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
