/**
 * @author Patrick Hoffmann
 * @author Marc Eisoldt
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

using namespace fastsense::map;

extern "C"
{

    void xi_to_transform(float xi[6], float transform[4][4])
    {
        float theta = hls_sqrt_approx(xi[0] * xi[0] + xi[1] * xi[1] + xi[2] * xi[2]);
        float sin_theta, cos_theta;
        hls_sincos(theta, &sin_theta, &cos_theta);
        cos_theta = 1 - cos_theta;

        xi[0] /= theta;
        xi[1] /= theta;
        xi[2] /= theta;
        float L[3][3] =
        {
            0, -xi[2], xi[1],
            xi[2], 0, -xi[0],
            -xi[1], xi[0], 0
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

    void registration_step(PointHW* pointData,
                           int numPoints,
                           IntTuple* mapData0,
                           IntTuple* mapData1,
                           IntTuple* mapData2,
                           const LocalMapHW& map,
                           int transform_matrix[4][4],
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
#pragma HLS loop_tripcount min=0 max=30000

            PointHW point_tmp = pointData[i];

            int point[3], point_mul[3];
#pragma HLS array_partition variable=point complete
#pragma HLS array_partition variable=point_mul complete
            point_mul[0] = point_tmp.x;
            point_mul[1] = point_tmp.y;
            point_mul[2] = point_tmp.z;

            //apply transform for point.
            fastsense::registration::transform_point(transform_matrix, point_mul, point);

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

    void krnl_reg(PointHW* pointData,
                  int numPoints,
                  IntTuple* mapData0,
                  IntTuple* mapData1,
                  IntTuple* mapData2,
                  int sizeX,   int sizeY,   int sizeZ,
                  int posX,    int posY,    int posZ,
                  int offsetX, int offsetY, int offsetZ,
                  int max_iterations,
                  int it_weight_gradient,
                  float* in_transform,
                  float* out_transform
                 )
    {
#pragma HLS INTERFACE m_axi port=pointData offset=slave bundle=scanmem latency=22
#pragma HLS INTERFACE m_axi port=mapData0 offset=slave bundle=mapmem0 latency=22
#pragma HLS INTERFACE m_axi port=mapData1 offset=slave bundle=mapmem1 latency=22
#pragma HLS INTERFACE m_axi port=mapData2 offset=slave bundle=mapmem2 latency=22
#pragma HLS INTERFACE m_axi port=in_transform offset=slave bundle=gmem latency=22
#pragma HLS INTERFACE m_axi port=out_transform offset=slave bundle=gmem latency=22

        LocalMapHW map{sizeX, sizeY, sizeZ,
                       posX, posY, posZ,
                       offsetX, offsetY, offsetZ};
        float alpha = 0.0f;
        //define local variables
        long h[6][6];
        long g[6];
        int int_transform[4][4]; // converted to int using MATRIX_RESOLUTION
        float next_transform[4][4]; // result of one iteration
        float total_transform[4][4]; // accumulated total transform
        float temp_transform[4][4]; // for matrix multiplication
        float previous_errors[4] = {0, 0, 0, 0};
#pragma HLS array_partition variable=h complete dim=0
#pragma HLS array_partition variable=g complete
#pragma HLS array_partition variable=int_transform complete dim=0
#pragma HLS array_partition variable=next_transform complete dim=0
#pragma HLS array_partition variable=total_transform complete dim=0
#pragma HLS array_partition variable=temp_transform complete dim=0
#pragma HLS array_partition variable=previous_errors complete

        long error, count;

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

    registration_loop:
        for (int i = 0; i < max_iterations; i++)
        {
#pragma HLS loop_tripcount min=200 max=200

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

            registration_step(pointData,
                              numPoints,
                              mapData0,
                              mapData1,
                              mapData2,
                              map,
                              int_transform,
                              h,
                              g,
                              error,
                              count
                             );

            float xi[6];
            // TODO: calc xi
            // Matrix6f hf = h.cast<float>();
            // Vector6f gf = g.cast<float>();
            // // W Matrix
            // hf += alpha * count * Matrix6f::Identity();
            // Vector6f xi = solve(hf, -gf);

            xi_to_transform(xi, next_transform);
            fastsense::registration::MatrixMul<float, 4, 4, 4>(next_transform, total_transform, temp_transform);

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

            constexpr float epsilon = 0.0001; // TODO: make parameter?
            if (fabsf(err - previous_errors[2]) < epsilon && fabsf(err - previous_errors[0]) < epsilon)
            {
                //std::cout << "Stopped after " << i << " / " << max_iterations_ << " Iterations" << std::endl;
                break;
            }
            for (int e = 1; e < 4; e++)
            {
#pragma HLS unroll
                previous_errors[e - 1] = previous_errors[e];
            }
            previous_errors[3] = err;
        }

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
    }
}
