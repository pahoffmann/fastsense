/**
 * @author Patrick Hoffmann
 * @author Marc Eisdoldt
 */

#include <map/local_map_hw.h>
#include <util/point_hw.h>
#include <registration/kernel/reg_hw.h>
#include <iostream>

struct IntTuple
{
    int first;
    int second;
};

// constexpr int MAP_SHIFT = 6;
// constexpr int MAP_RESOLUTION = 64;
// constexpr int MATRIX_RESOLUTION = 1024;

extern "C"
{

    void krnl_reg(PointHW* pointData,
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
                  int* transform,
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

        for (int row = 0; row < 6; row++)
        {
#pragma HLS unroll
            for (int col = 0; col < 6; col++)
            {
#pragma HLS unroll
                local_h[row][col] = 0;
            }
            local_g[row] = 0;
        }

        long local_error = 0;
        long local_count = 0;

                    
        int transform_matrix[4][4];

    transform_loop:
        for (int row = 0; row < 4; row++)
        {
#pragma HLS unroll
            for (int col = 0; col < 4; col++)
            {
#pragma HLS unroll
                transform_matrix[row][col] = transform[row * 4 + col]; //TODO: check indexing
            }         
        }

    point_loop:
        for (int i = 0; i < numPoints; i++)
        {
#pragma HLS loop_tripcount min=0 max=30000

            PointHW point_tmp = pointData[i];

            //TODO: check if copy is needed
            int point[4][1], point_mul[4][1];
            point_mul[0][0] = point_tmp.x;
            point_mul[1][0] = point_tmp.y;
            point_mul[2][0] = point_tmp.z;
            point_mul[3][0] = 1;

            //apply transform for point.
            fastsense::registration::MatrixMulTransform(transform_matrix, point_mul, point);

            //revert matrix resolution step => point has real data
            point[0][0] /= MATRIX_RESOLUTION;
            point[1][0] /= MATRIX_RESOLUTION;
            point[2][0] /= MATRIX_RESOLUTION;
            point[3][0] /= MATRIX_RESOLUTION;

            PointHW buf;
            buf.x = point[0][0] / MAP_RESOLUTION;
            buf.y = point[1][0] / MAP_RESOLUTION;
            buf.z = point[2][0] / MAP_RESOLUTION;
            buf.dummy = 1;

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

            long cross_p_x = static_cast<long>(point[1][0]) * gradient[2] - static_cast<long>(point[2][0]) * gradient[1];
            long cross_p_y = static_cast<long>(point[2][0]) * gradient[0] - static_cast<long>(point[0][0]) * gradient[2];
            long cross_p_z = static_cast<long>(point[0][0]) * gradient[1] - static_cast<long>(point[1][0]) * gradient[0];

            long jacobi[6];
#pragma HLS ARRAY_RESHAPE variable=jacobi complete

            jacobi[0] = cross_p_x;
            jacobi[1] = cross_p_y;
            jacobi[2] = cross_p_z;
            jacobi[3] = gradient[0];
            jacobi[4] = gradient[1];
            jacobi[5] = gradient[2];

            //add multiplication result to local_h
            //TODO: pragmas

        local_h_loop:
            for (int row = 0; row < 6; row++)
            {
#pragma HLS unroll
                for (int col = 0; col < 6; col++)
                {
#pragma HLS unroll
                    // local_h += jacobi * jacobi.transpose()
                    // transpose === swap row and column
                    local_h[row][col] += jacobi[row] * jacobi[col];
                }
            }

            //TODO: ADD PRAGMATA
        local_g_loop:
            for (int count = 0; count < 6; count++)
            {
#pragma HLS unroll

                local_g[count] += jacobi[count] * current.first;
            }

            local_error += abs(current.first);
            local_count++;
        }

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
