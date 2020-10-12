/**
 * @author Patrick Hoffmann
 * @author Marc Eisdoldt
 */

#include <map/local_map_hw.h>
#include <msg/point.h>
#include <registration/reg_hw.h>
#include <iostream>

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
    int dummy;
};

constexpr int MAP_SHIFT = 6;

extern "C"
{

    void krnl_reg(Point* pointData,
                  int numPoints,
                  IntTuple* mapData0,
                  IntTuple* mapData1,
                  IntTuple* mapData2,
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
#pragma HLS INTERFACE m_axi port=pointData offset=slave bundle=scanmem latency=8
#pragma HLS INTERFACE m_axi port=mapData0 offset=slave bundle=mapmem0 latency=8
#pragma HLS INTERFACE m_axi port=mapData1 offset=slave bundle=mapmem1 latency=8
#pragma HLS INTERFACE m_axi port=mapData2 offset=slave bundle=mapmem2 latency=8

        fastsense::map::LocalMapHW map{sizeX, sizeY, sizeZ,
                                       posX, posY, posZ,
                                       offsetX, offsetY, offsetZ};
        //define local variables
        long local_h[6][6];
        long local_g[6];

#pragma HLS ARRAY_RESHAPE variable=local_h complete dim=0
#pragma HLS ARRAY_RESHAPE variable=local_g complete

        local_h[0][0] = 0;
        local_h[0][1] = 0;
        local_h[0][2] = 0;
        local_h[0][3] = 0;
        local_h[0][4] = 0;
        local_h[0][5] = 0;

        local_h[1][0] = 0;
        local_h[1][1] = 0;
        local_h[1][2] = 0;
        local_h[1][3] = 0;
        local_h[1][4] = 0;
        local_h[1][5] = 0;

        local_h[2][0] = 0;
        local_h[2][1] = 0;
        local_h[2][2] = 0;
        local_h[2][3] = 0;
        local_h[2][4] = 0;
        local_h[2][5] = 0;

        local_h[3][0] = 0;
        local_h[3][1] = 0;
        local_h[3][2] = 0;
        local_h[3][3] = 0;
        local_h[3][4] = 0;
        local_h[3][5] = 0;

        local_h[4][0] = 0;
        local_h[4][1] = 0;
        local_h[4][2] = 0;
        local_h[4][3] = 0;
        local_h[4][4] = 0;
        local_h[4][5] = 0;

        local_h[5][0] = 0;
        local_h[5][1] = 0;
        local_h[5][2] = 0;
        local_h[5][3] = 0;
        local_h[5][4] = 0;
        local_h[5][5] = 0;

        local_g[0] = 0;
        local_g[1] = 0;
        local_g[2] = 0;
        local_g[3] = 0;
        local_g[4] = 0;
        local_g[5] = 0;

        long local_error = 0;
        long local_count = 0;

    point_loop:
        for (int i = 0; i < numPoints; i++)
        {
#pragma HLS loop_tripcount min=0 max=30000

            auto point = pointData[i];

            //if(point.x == 0 && point.y == 0 && point.z == 0) continue; //hacky af. TODO: better solution by pascal


            //TODO: if transform needs to take place in hw, here is the place ;)

            Point buf;
            buf.x = point.x >> MAP_SHIFT;
            buf.y = point.y >> MAP_SHIFT;
            buf.z = point.z >> MAP_SHIFT;

            //get value of local map
            const auto& current = map.get(mapData0, buf.x, buf.y, buf.z);

            if (current.second == 0)
            {
                continue;
            }

            int gradient[3];

#pragma HLS ARRAY_RESHAPE variable=gradient complete

            gradient[0] = 0;
            gradient[1] = 0;
            gradient[2] = 0;

        gradient_loop:
            for (size_t axis = 0; axis < 3; axis++)
            {
#pragma HLS unroll

                int index[3] = {buf.x, buf.y, buf.z};
                index[axis] -= 1;
                const auto last = map.get(mapData1, index[0], index[1], index[2]);
                index[axis] += 2;
                const auto next = map.get(mapData2, index[0], index[1], index[2]);
                if (last.second != 0 && next.second != 0 && (next.first > 0.0) == (last.first > 0.0))
                {
                    gradient[axis] = (next.first - last.first) / 2;
                }
            }

            long cross_p_x = static_cast<long>(point.y) * gradient[2] - static_cast<long>(point.z) * gradient[1];
            long cross_p_y = static_cast<long>(point.z) * gradient[0] - static_cast<long>(point.x) * gradient[2];
            long cross_p_z = static_cast<long>(point.x) * gradient[1] - static_cast<long>(point.y) * gradient[0];

            long jacobi[6][1];
            long jacobi_transpose[1][6];

            jacobi_transpose[0][0] = cross_p_x;
            jacobi_transpose[0][1] = cross_p_y;
            jacobi_transpose[0][2] = cross_p_z;
            jacobi_transpose[0][3] = gradient[0];
            jacobi_transpose[0][4] = gradient[1];
            jacobi_transpose[0][5] = gradient[2];

            jacobi[0][0] = cross_p_x;
            jacobi[1][0] = cross_p_y;
            jacobi[2][0] = cross_p_z;
            jacobi[3][0] = gradient[0];
            jacobi[4][0] = gradient[1];
            jacobi[5][0] = gradient[2];

            long tmp[6][6];

            //std::cout << "Kernel_CPP " << __LINE__ << std::endl;

            fastsense::registration::MatrixMul(jacobi, jacobi_transpose, tmp);

            //std::cout << "Kernel_CPP " << __LINE__ << std::endl;

            //add multiplication result to local_h
            //TODO: pragmas

        local_h_loop:
            for (int row = 0; row < 6; row++)
            {
#pragma HLS unroll

                for (int col = 0; col < 6; col++)
                {
#pragma HLS unroll
                    local_h[row][col] += tmp[row][col];
                }
            }

            //TODO: ADD PRAGMATA
        local_g_loop:
            for (int count = 0; count < 6; count++)
            {
#pragma HLS unroll

                local_g[count] += jacobi[count][0] * current.first;
            }

            local_error += abs(current.first);
            local_count++;

            //if(i == 0) std::cout << "Kernel_CPP Iterating trough points......." << __LINE__ << std::endl;

        }

        //std::cout << "Kernel_CPP " << __LINE__ << std::endl;

        //fill output buffer.
    out_row_loop:
        for (int row = 0; row < 6; row++)
        {
#pragma HLS unroll

        out_col_loop:
            for (int col = 0; col < 6; col++)
            {
#pragma HLS unroll

                outbuf[row + col * 6] = local_h[row][col]; //from 0 to 35: local_h
            }

            outbuf[36 + row] = local_g[row]; //from 36 to 41: local_g
        }

        outbuf[42] = local_error;
        outbuf[43] = local_count;
    }
}
