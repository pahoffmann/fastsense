#pragma once

/**
 * @file reg_hw.h
 * @author Marc Eisoldt (meisoldt@uos.de)
 * @brief
 * @version 0.1
 * @date 2020-10-05
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <cmath>

#define MAT_A_ROWS 6
#define MAT_A_COLS 1
#define MAT_B_ROWS 1
#define MAT_B_COLS 6

typedef long mat_a_t;
typedef long mat_b_t;
typedef long result_t; // select the required bit width

namespace fastsense
{

namespace registration
{

// Prototype of top level function for C-synthesis
void MatrixMul(
    mat_a_t a[MAT_A_ROWS][MAT_A_COLS],
    mat_b_t b[MAT_B_ROWS][MAT_B_COLS],
    result_t res[MAT_A_ROWS][MAT_B_COLS])
{
#pragma HLS INLINE

    // Iterate over the rows of the A matrix
    for (int i = 0; i < MAT_A_ROWS; i++)
    {
#pragma HLS unroll

        // Iterate over the columns of the B Matrix
        for (int j = 0; j < MAT_B_COLS; j++)
        {
#pragma HLS unroll
            res[i][j] = 0;

            for (int k = 0; k < MAT_B_ROWS; k++)
            {
#pragma HLS unroll

                res[i][j] += (result_t)(a[i][k]) * (result_t)(b[k][j]);
            }
        }
    }
}

//TODO: template this..
void MatrixMulTransform(
    int a[4][4],
    int b[4][1],
    int res[4][1])
{
#pragma HLS INLINE

    // Iterate over the rows of the A matrix
    for (int i = 0; i < 4; i++)
    {
#pragma HLS unroll

        // Iterate over the columns of the B Matrix
        for (int j = 0; j < 1; j++)
        {
#pragma HLS unroll
            res[i][j] = 0;

            for (int k = 0; k < 4; k++)
            {
#pragma HLS unroll

                res[i][j] += (result_t)(a[i][k]) * (result_t)(b[k][j]);
            }
        }
    }
}

} // namespace registration

} // namespace fastsense
