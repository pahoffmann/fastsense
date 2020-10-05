/**
 * @author Marcel Flottmann
 */

#include <map/local_map_hw.h>
#include <msg/point.h>
#include <registration/reg_hw.h>
#include <stdexcept> 
#include <util/logging/logger.h>

struct IntTuple
{
    int first;
    int second;
};

struct Point
{
    int x;
    int y;
    int z;
};

extern "C"
{

    void krnl_local_map_test(Point* pointData,
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
                             int mapResolution,
                             long* outbuf
                            )
    {

#pragma HLS DATA_PACK variable=pointData
#pragma HLS INTERFACE m_axi port=pointData offset=slave bundle=gmem
#pragma HLS INTERFACE s_axilite port=scanPoints bundle=control
#pragma HLS INTERFACE s_axilite port=numPoints bundle=control

#pragma HLS DATA_PACK variable=mapData
#pragma HLS INTERFACE m_axi port=mapData offset=slave bundle=gmem
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
#pragma HLS INTERFACE s_axilite port=mapResolution bundle=control

#pragma HLS INTERFACE s_axilite port = outbuf bundle = control //outbuf

#pragma HLS INTERFACE s_axilite port=return bundle=control

        fastsense::map::LocalMapHW map{sizeX, sizeY, sizeZ,
                                       posX, posY, posZ,
                                       offsetX, offsetY, offsetZ};
        //define local variables
        static long local_h[6][6] = {0};
        static long local_g[6] = {0};
        long local_error = 0;
        long local_count = 0;

        for (int i = 0; i < numPoints; i += sizeof(Point))
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

                static int gradient[3] = {};

                for (size_t axis = 0; axis < 3; axis++)
                {
                    static int index[3] = {point.x / mapResolution, point.y / mapResolution, point.z / mapResolution};
                    index[axis] -= 1;
                    const auto last = map.get(mapData, index[0], index[1], index[2]);
                    index[axis] += 2;
                    const auto next = map.get(mapData, index[0], index[1], index[2]);
                    if (last.second != 0 && next.second != 0 && (next.first > 0.0) == (last.first > 0.0))
                    {
                        gradient[axis] = (next.first - last.first) / 2;
                    }
                }
                

                //calculate cross_product
                static long cross_p[3] = {point.y * gradient[2] - point.z * gradient[1],
                                         point.z * gradient[0] - point.x * gradient[2],
                                         point.x * gradient[1] - point.y * gradient[0]};

                static long jacobi[6][1] = {cross_p[0], cross_p[1], cross_p[2], gradient[0], gradient[1], gradient[2]};
                
                //TODO: ADD PRAGMATA
                for(int i = 0; i < 6; i++)
                {
                    local_g[i] += jacobi[i][0] * current.first;
                }

                static long jacobi_transpose[1][6] = {jacobi[0][0],
                                                        jacobi[1][0],
                                                        jacobi[2][0],
                                                        jacobi[3][0],
                                                        jacobi[4][0],
                                                        jacobi[5][0]};

                static long tmp[6][6];
                fastsense::registration::MatrixMul(jacobi, jacobi_transpose, tmp);
                
                //add multiplication result to local_h
                //TODO: pragmas 
                for(int row = 0; row < 6; row++)
                {
                    for(int col = 0; col < 5; col++)
                    {
                        local_h[row][col] += tmp[row][col];
                    }
                }

                local_error += abs(current.first);
                local_count++;
            }
            catch (const std::out_of_range& oor)
            {
                fastsense::util::logging::Logger::error("Out of range exception in reg kernel: %s", oor.what());
            }
        

        }

        //TODO: fill output buffer.
        for(int row = 0; row < 6; row++)
        {
            for(int col = 0; col < 6; col++)
            {
                outbuf[row * 6 + col] = local_h[row][col]; //from 0 to 35: local_h
            }

            outbuf[36 + row] = local_g[row]; //from 36 to 41: local_g
        }

        outbuf[42] = local_error;
        outbuf[43] = local_count;
    }
}