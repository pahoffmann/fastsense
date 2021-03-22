/**
 * @author Patrick Hoffmann
 * @author Marc Eisoldt
 * @author Malte Hillmann
 */

#include <map/local_map_hw.h>
#include <util/point_hw.h>
#include <util/tsdf_hw.h>
#include <registration/kernel/reg_hw.h>
#include <registration/kernel/linear_solver.h>
#include <iostream>

using namespace fastsense::map;
using namespace fastsense::registration;

constexpr int NUM_POINTS = 6000;
constexpr int SPLIT_FACTOR = 3; // MARKER: SPLIT

extern "C"
{

    /**
     * @brief Convert the motion representation into a transformation matrix
     * 
     * @param xi        Motion representation, which should be converted into a transformatio matrix.
     *                  It is represented as a 6D-vector with three linear velocity and three angular velocity entries (for every axis) 
     * @param transform Transformation matrix built from the motion representation
     * @param center    Current center of the scan in global coordinates
     */
    void xi_to_transform(const float xi[6], float transform[4][4], PointHW& center)
    {
        // Formula 3.9 on Page 40 of "Truncated Signed Distance Fields Applied To Robotics"

        // Rotation around an Axis.
        // Direction of axis (l) = Direction of angular_velocity
        // Angle of Rotation (theta) = Length of angular_velocity
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

        // translation (added later)
        transform[0][3] = 0;
        transform[1][3] = 0;
        transform[2][3] = 0;

        // bottom row
        transform[3][0] = 0;
        transform[3][1] = 0;
        transform[3][2] = 0;
        transform[3][3] = 1;

        // apply the rotation around the old center
        // => rotate around Point = shift Point to origin -> rotate -> shift back to Point

        // shift old_center to origin
        float old_center[3];
        old_center[0] = -center.x;
        old_center[1] = -center.y;
        old_center[2] = -center.z;

        // rotate
        float shift[3];
        transform_point(transform, old_center, shift);

        // shift back to center + new translation
        transform[0][3] = shift[0] + center.x + xi[3];
        transform[1][3] = shift[1] + center.y + xi[4];
        transform[2][3] = shift[2] + center.z + xi[5];
    }

    /**
     * @brief Perfom a registration iteration step, where a H and g matrix is built based 
     *        on every scan point which can be used to determine an intermediat transformation
     * 
     * @param pointData        Current set of scan points, which should matched against the map
     * @param numPoints        Number of points in the current scan
     * @param mapData0         First map data reference, which is used to get the TSDF value of the grid the points lie in
     * @param mapData1         Second map data reference, which is used to get rear TSDF value of an axis around a point to build a TSDF gradient
     * @param mapData2         Third map data reference, which is used to get front TSDF value of an axis around a point to build a TSDF gradient
     * @param map              Map representation to access the raw map data
     * @param transform_matrix Current total transformation
     * @param center           Current center of the scan in global coordinates
     * @param h                H matrix, which is built from the scan points and the map
     * @param g                g matrix, which is built from the scan points and the map
     * @param error            Determined registration error of the scan points to the map 
     * @param count            Number of point, which were considered for the registration step
     */
    void registration_step(const PointHW* pointData,
                           int numPoints,
                           TSDFEntryHW* mapData0, TSDFEntryHW* mapData1, TSDFEntryHW* mapData2,
                           const LocalMapHW& map,
                           const int transform_matrix[4][4],
                           const PointHW& center,
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

            point[0] -= center.x;
            point[1] -= center.y;
            point[2] -= center.z;

            //get value of local map
            const auto& current = map.get(mapData0, buf.x, buf.y, buf.z);

            // If the TSDF value was not set, we have no information about it and so the point cannot be considered for registration
            if (current.weight == 0)
            {
                continue;
            }

            int gradient[3] = {0, 0, 0};
#pragma HLS array_partition variable=gradient complete

        // Build the TSDF gradient based on the local TSDF neighborhood of a point
        gradient_loop:
            for (size_t axis = 0; axis < 3; axis++)
            {
#pragma HLS unroll
                int index[3] = {buf.x, buf.y, buf.z};

                index[axis] -= 1;
                const auto last = map.get(mapData1, index[0], index[1], index[2]);
                index[axis] += 2;
                const auto next = map.get(mapData2, index[0], index[1], index[2]);
                if (last.weight != 0 && next.weight != 0 && (next.value > 0) == (last.value > 0))
                {
                    gradient[axis] = (next.value - last.value) / 2;
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
                g[row] += jacobi[row] * current.value;
            }

            error += hls_abs(current.value);
            count++;
        }
    }

// Declares the parameters for variables ending with 'n'. Macro Syntax: ## means concat variable name with the contents of 'n'
#define DECLARE_PARAMS(n) \
    const PointHW* pointData##n, \
    TSDFEntryHW* mapData##n##0, TSDFEntryHW* mapData##n##1, TSDFEntryHW* mapData##n##2, \
    long h##n[6][6], long g##n[6], long& error##n, long& count##n

#define USE_PARAMS(n) \
    pointData##n, \
    mapData##n##0, mapData##n##1, mapData##n##2, \
    h##n, g##n, error##n, count##n

    /**
     * @brief Create a dataflow from the registration process 
     * 
     */
    void reg_dataflow(DECLARE_PARAMS(0), // MARKER: SPLIT
                      DECLARE_PARAMS(1),
                      DECLARE_PARAMS(2),
                      int step,
                      int last_step,
                      const LocalMapHW& map,
                      const int transform_matrix[4][4],
                      const PointHW& center
                     )
    {
#pragma HLS dataflow

#define CALL_STEP(n, STEP_SIZE) \
        registration_step(pointData##n + n * step, STEP_SIZE, \
                          mapData##n##0, mapData##n##1, mapData##n##2, \
                          map, \
                          transform_matrix, center, \
                          h##n, g##n, error##n, count##n);

        CALL_STEP(0, step); // MARKER: SPLIT
        CALL_STEP(1, step);
        CALL_STEP(2, last_step);
    }

    /**
     * @brief Perform a complete registration of the given point data to the current TSDF map
     *  
     * @param pointDataX Point data assigned to different memory points, which should be registered
     * @param numPoints  Number of points in the current scan
     * @param mapDataXX  Map data assigned to different memory ports. which should be used to register the scan points
     * @param sizeX Map size in x direction
     * @param sizeY Map size in y direction
     * @param sizeZ Map size in z direction
     * @param posX  X coordinate of the scanner in the global system
     * @param posY  Y coordinate of the scanner in the global system
     * @param posZ  Z coordinate of the scanner in the global system
     * @param offsetX Offset for the x axis regarding the current map shift 
     * @param offsetY Offset for the y axis regarding the current map shift
     * @param offsetZ Offset for the z axis regarding the current map shift
     * @param max_iterations Maximum number of iteration for determining the transfomormation of the current scan to the map
     * @param it_weight_gradient Decay variable for the iteration influence
     * @param in_transform Initial transformation for scan point
     * @param out_transform Result transformation for the scan into the global coordinate system
     */
    void krnl_reg(const PointHW* pointData0, // MARKER: SPLIT
                  const PointHW* pointData1,
                  const PointHW* pointData2,
                  int numPoints,
                  TSDFEntryHW* mapData00, TSDFEntryHW* mapData01, TSDFEntryHW* mapData02, // MARKER: SPLIT
                  TSDFEntryHW* mapData10, TSDFEntryHW* mapData11, TSDFEntryHW* mapData12,
                  TSDFEntryHW* mapData20, TSDFEntryHW* mapData21, TSDFEntryHW* mapData22,
                  int sizeX,   int sizeY,   int sizeZ,
                  int posX,    int posY,    int posZ,
                  int offsetX, int offsetY, int offsetZ,
                  int max_iterations,
                  float it_weight_gradient,
                  float* in_transform,
                  float* out_transform
                 )
    {
        // MARKER: SPLIT
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=pointData0    bundle=pointData0mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData00     bundle=mapData00mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData01     bundle=mapData01mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData02     bundle=mapData02mem

#pragma HLS INTERFACE m_axi latency=22 offset=slave port=pointData1    bundle=pointData1mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData10     bundle=mapData10mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData11     bundle=mapData11mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData12     bundle=mapData12mem

#pragma HLS INTERFACE m_axi latency=22 offset=slave port=pointData2    bundle=pointData2mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData20     bundle=mapData20mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData21     bundle=mapData21mem
#pragma HLS INTERFACE m_axi latency=22 offset=slave port=mapData22     bundle=mapData22mem

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

        // MARKER: SPLIT
        long h0[6][6], g0[6], error0, count0;
#pragma HLS array_partition complete variable=h0
#pragma HLS array_partition complete variable=g0

        long h1[6][6], g1[6], error1, count1;
#pragma HLS array_partition complete variable=h1
#pragma HLS array_partition complete variable=g1

        long h2[6][6], g2[6], error2, count2;
#pragma HLS array_partition complete variable=h2
#pragma HLS array_partition complete variable=g2

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
        float epsilon = in_transform[16];

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
            std::cout << "\r" << i + 1 << " / " << max_iterations << std::flush;
#endif

            // "center" of Scan == estimated Position of Scanner == translation in Pose
            PointHW center(total_transform[0][3],
                           total_transform[1][3],
                           total_transform[2][3]);

            reg_dataflow(USE_PARAMS(0), // MARKER: SPLIT
                         USE_PARAMS(1),
                         USE_PARAMS(2),
                         step,
                         last_step,
                         map,
                         int_transform,
                         center
                        );

            // MARKER: SPLIT
            error0 += error1 + error2;
            count0 += count1 + count2;

            float alpha_bonus = alpha * count0;

            for (int row = 0; row < 6; row++)
            {
#pragma HLS unroll
                for (int col = 0; col < 6; col++)
                {
#pragma HLS unroll
                    // MARKER: SPLIT
                    h_float[row][col] = static_cast<float>(h0[row][col] + h1[row][col] + h2[row][col]);
                }
                // MARKER: SPLIT
                g_float[row] = static_cast<float>(-(g0[row] + g1[row] + g2[row]));

                h_float[row][row] += alpha_bonus;
            }

            // Invert H matrix und multiplicate it with g to receive the next motion
            lu_decomposition<float, 6>(h_float);
            lu_solve<float, 6>(h_float, g_float, xi);

            // Convert the current motion iterion into a transformation matrix and add it to the total transformation
            xi_to_transform(xi, next_transform, center);
            MatrixMul<float, 4, 4, 4>(next_transform, total_transform, temp_transform);

            // copy temp_transform to total_transform
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
            float err = (float)error0 / count0;
            float d1 = err - previous_errors[2];
            float d2 = err - previous_errors[0];

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
