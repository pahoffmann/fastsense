/**
 * @author Patrick Hoffmann
 * @author Marc Eisoldt
 * @author Malte Hillmann
 */

#include <map/local_map_hw.h>
#include <util/point_hw.h>
#include <registration/kernel/reg_hw.h>
#include <registration/kernel/linear_solver.h>
#include <iostream>

struct IntTuple
{
    int first;
    int second;
};

using namespace fastsense::map;
using namespace fastsense::registration;

constexpr int NUM_POINTS = 6000;
constexpr int SPLIT_FACTOR = 2;

extern "C"
{

    void xi_to_transform(const float xi[6], float transform[4][4])
    {
        float theta = hls_sqrt_float(xi[0] * xi[0] + xi[1] * xi[1] + xi[2] * xi[2]);
        float sin_theta, cos_theta;
        hls_sincos(theta, &sin_theta, &cos_theta);
        cos_theta = 1 - cos_theta;

        float xi_0 = xi[0] / theta;
        float xi_1 = xi[1] / theta;
        float xi_2 = xi[2] / theta;
        float L[3][3] =
        {
            {0, -xi_2, xi_1},
            {xi_2, 0, -xi_0},
            {-xi_1, xi_0, 0}
        };

        for (int row = 0; row < 3; row++)
        {
#pragma HLS unroll
            for (int col = 0; col < 3; col++)
            {
#pragma HLS unroll
                transform[row][col] = sin_theta * L[row][col];
                for (int k = 0; k < 3; k++)
                {
#pragma HLS unroll
                    transform[row][col] += cos_theta * L[row][k] * L[k][col];
                }
            }
        }

        // identity diagonal
        transform[0][0] += 1;
        transform[1][1] += 1;
        transform[2][2] += 1;

        // translation
        transform[0][3] = xi[3];
        transform[1][3] = xi[4];
        transform[2][3] = xi[5];

        // bottom row
        transform[3][0] = 0;
        transform[3][1] = 0;
        transform[3][2] = 0;
        transform[3][3] = 1;
    }

    void registration_step(const PointHW* pointData,
                           int numPoints,
                           IntTuple* mapData0, IntTuple* mapData1, IntTuple* mapData2,
                           const LocalMapHW& map,
                           const int transform_matrix[4][4],
                           long h[6][6],
                           long g[6],
                           long& error,
                           long& count
                          )
    {
        for (int row = 0; row < 6; row++)
        {
#pragma HLS unroll
            for (int col = 0; col < 6; col++)
            {
#pragma HLS unroll
                h[row][col] = 0;
            }
            g[row] = 0;
        }
        error = 0;
        count = 0;

    point_loop:
        for (int i = 0; i < numPoints; i++)
        {
#pragma HLS loop_tripcount min=NUM_POINTS/SPLIT_FACTOR max=NUM_POINTS/SPLIT_FACTOR
#pragma HLS pipeline II=3

            PointHW point_tmp = pointData[i];

            int point[3], point_mul[3];
#pragma HLS array_partition variable=point complete
#pragma HLS array_partition variable=point_mul complete
            point_mul[0] = point_tmp.x;
            point_mul[1] = point_tmp.y;
            point_mul[2] = point_tmp.z;

            //apply transform for point.
            transform_point(transform_matrix, point_mul, point);

            //revert matrix resolution step => point has real data
            point[0] /= MATRIX_RESOLUTION;
            point[1] /= MATRIX_RESOLUTION;
            point[2] /= MATRIX_RESOLUTION;

            PointHW buf;
            buf.x = point[0] / MAP_RESOLUTION;
            buf.y = point[1] / MAP_RESOLUTION;
            buf.z = point[2] / MAP_RESOLUTION;

            //get value of local map
            const auto& current = map.get(mapData0, buf.x, buf.y, buf.z);

            if (current.second == 0)
            {
                continue;
            }

            int gradient[3] = {0, 0, 0};
#pragma HLS array_partition variable=gradient complete

        gradient_loop:
            for (size_t axis = 0; axis < 3; axis++)
            {
#pragma HLS unroll
                int index[3] = {buf.x, buf.y, buf.z};

                index[axis] -= 1;
                const auto last = map.get(mapData1, index[0], index[1], index[2]);
                index[axis] += 2;
                const auto next = map.get(mapData2, index[0], index[1], index[2]);
                if (last.second != 0 && next.second != 0 && (next.first > 0) == (last.first > 0))
                {
                    gradient[axis] = (next.first - last.first) / 2;
                }
            }

            long jacobi[6];
#pragma HLS array_partition variable=jacobi complete

            // cross product point x gradient
            jacobi[0] = static_cast<long>(point[1]) * gradient[2] - static_cast<long>(point[2]) * gradient[1];
            jacobi[1] = static_cast<long>(point[2]) * gradient[0] - static_cast<long>(point[0]) * gradient[2];
            jacobi[2] = static_cast<long>(point[0]) * gradient[1] - static_cast<long>(point[1]) * gradient[0];
            jacobi[3] = gradient[0];
            jacobi[4] = gradient[1];
            jacobi[5] = gradient[2];

            //add multiplication result to h

            for (int row = 0; row < 6; row++)
            {
#pragma HLS unroll
                for (int col = 0; col < 6; col++)
                {
#pragma HLS unroll
                    // h += jacobi * jacobi.transpose()
                    // transpose === swap row and column
                    h[row][col] += jacobi[row] * jacobi[col];
                }
                g[row] += jacobi[row] * current.first;
            }

            error += hls_abs(current.first);
            count++;
        }
    }

    void reg_dataflow(const PointHW* pointData0,
                      const PointHW* pointData1,
                      int step,
                      int last_step,
                      IntTuple* mapData00, IntTuple* mapData01, IntTuple* mapData02,
                      IntTuple* mapData10, IntTuple* mapData11, IntTuple* mapData12,
                      const LocalMapHW& map,
                      const int transform_matrix[4][4],
                      long h0[6][6], long g0[6], long& error0, long& count0,
                      long h1[6][6], long g1[6], long& error1, long& count1
                     )
    {
#pragma HLS dataflow

        registration_step(pointData0, step,
                          mapData00, mapData01, mapData02,
                          map,
                          transform_matrix,
                          h0, g0, error0, count0);

        registration_step(pointData1, last_step,
                          mapData10, mapData11, mapData12,
                          map,
                          transform_matrix,
                          h1, g1, error1, count1);
    }

    void krnl_reg(const PointHW* pointData0,
                  const PointHW* pointData1,
                  int numPoints,
                  IntTuple* mapData00, IntTuple* mapData01, IntTuple* mapData02,
                  IntTuple* mapData10, IntTuple* mapData11, IntTuple* mapData12,
                  int sizeX,   int sizeY,   int sizeZ,
                  int posX,    int posY,    int posZ,
                  int offsetX, int offsetY, int offsetZ,
                  int max_iterations,
                  float it_weight_gradient,
                  float* in_transform,
                  float* out_transform
                 )
    {
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=pointData0    bundle=pointData0mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData00     bundle=mapData00mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData01     bundle=mapData01mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData02     bundle=mapData02mem

#pragma HLS INTERFACE m_axi latency=22 offset=slave port=pointData1    bundle=pointData1mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData10     bundle=mapData10mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData11     bundle=mapData11mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData12     bundle=mapData12mem

#pragma HLS INTERFACE m_axi latency=22 offset=slave port=in_transform  bundle=transformmem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=out_transform bundle=transformmem

        LocalMapHW map{sizeX, sizeY, sizeZ,
                       posX, posY, posZ,
                       offsetX, offsetY, offsetZ};
        float alpha = 0.0f;
        //define local variables
        int int_transform[4][4]; // converted to int using MATRIX_RESOLUTION
        float next_transform[4][4]; // result of one iteration
        float total_transform[4][4]; // accumulated total transform
        float temp_transform[4][4]; // for matrix multiplication
        float previous_errors[4] = {0, 0, 0, 0};
        float h_float[6][6];
        float g_float[6];
        float xi[6];
#pragma HLS array_partition complete variable=int_transform
#pragma HLS array_partition complete variable=next_transform
#pragma HLS array_partition complete variable=total_transform
#pragma HLS array_partition complete variable=temp_transform
#pragma HLS array_partition complete variable=previous_errors
#pragma HLS array_partition complete variable=h_float
#pragma HLS array_partition complete variable=g_float
#pragma HLS array_partition complete variable=xi

        long h[6][6];
        long g[6];
        long error, count;
#pragma HLS array_partition complete variable=h
#pragma HLS array_partition complete variable=g

        long h1[6][6];
        long g1[6];
        long error1, count1;
#pragma HLS array_partition complete variable=h1
#pragma HLS array_partition complete variable=g1

        // Split variables
        int step = numPoints / SPLIT_FACTOR;
        int last_step = numPoints - step * (SPLIT_FACTOR - 1);

        // Pipeline to save ressources
    in_transform_loop:
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
#pragma HLS pipeline II=1
                total_transform[row][col] = in_transform[row * 4 + col];
            }
        }

        int i;
    registration_loop:
        for (i = 0; i < max_iterations; i++)
        {
#pragma HLS loop_tripcount min=200 max=200
#pragma HLS pipeline off

            // convert total_transform to int_transform
            for (int row = 0; row < 4; row++)
            {
#pragma HLS unroll
                for (int col = 0; col < 4; col++)
                {
#pragma HLS unroll
                    int_transform[row][col] = static_cast<int>(total_transform[row][col] * MATRIX_RESOLUTION);
                }
            }

#ifndef __SYNTHESIS__
            std::cout << "\r" << i << " / " << max_iterations << std::flush;
#endif

            reg_dataflow(pointData0,
                         pointData1 + step,
                         step,
                         last_step,
                         mapData00, mapData01, mapData02,
                         mapData10, mapData11, mapData12,
                         map,
                         int_transform,
                         h, g, error, count,
                         h1, g1, error1, count1
                        );

            error += error1;
            count += count1;

            float alpha_bonus = alpha * count;

            for (int row = 0; row < 6; row++)
            {
#pragma HLS unroll
                for (int col = 0; col < 6; col++)
                {
#pragma HLS unroll
                    h_float[row][col] = static_cast<float>(h[row][col] + h1[row][col]);
                }
                g_float[row] = static_cast<float>(-(g[row] + g1[row]));

                h_float[row][row] += alpha_bonus;
            }

            lu_decomposition<float, 6>(h_float);
            lu_solve<float, 6>(h_float, g_float, xi);

            xi_to_transform(xi, next_transform);
            MatrixMul<float, 4, 4, 4>(next_transform, total_transform, temp_transform);

            // copy temp_transform to total_transform
            // TODO: multiply in-place to avoid copy?
            for (int row = 0; row < 4; row++)
            {
#pragma HLS unroll
                for (int col = 0; col < 4; col++)
                {
#pragma HLS unroll
                    total_transform[row][col] = temp_transform[row][col];
                }
            }

            alpha += it_weight_gradient;
            float err = (float)error / count;
            float d1 = err - previous_errors[2];
            float d2 = err - previous_errors[0];

            constexpr float epsilon = 0.0001; // TODO: make parameter?
            if (d1 >= -epsilon && d1 <= epsilon && d2 >= -epsilon && d2 <= epsilon)
            {
                break;
            }
            for (int e = 1; e < 4; e++)
            {
#pragma HLS unroll
                previous_errors[e - 1] = previous_errors[e];
            }
            previous_errors[3] = err;
        }
#ifndef __SYNTHESIS__
        std::cout << std::endl;
#endif

        // Pipeline to save ressources
    out_transform_loop:
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
#pragma HLS pipeline II=1
                out_transform[row * 4 + col] = total_transform[row][col];
            }
        }
        out_transform[16] = i;
    }
}
