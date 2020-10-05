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

#define MAT_A_ROWS 4
#define MAT_A_COLS 4
#define MAT_B_ROWS 4
#define MAT_B_COLS 4

typedef int mat_a_t;
typedef int mat_b_t;
typedef int result_t; // select the required bit width

// Prototype of top level function for C-synthesis
void MatrixMul(
      mat_a_t a[MAT_A_ROWS][MAT_A_COLS],
      mat_b_t b[MAT_B_ROWS][MAT_B_COLS],
      result_t res[MAT_A_ROWS][MAT_B_COLS])
{
    #pragma HLS INLINE

    #pragma HLS ARRAY_RESHAPE variable=a complete dim=0
	#pragma HLS ARRAY_RESHAPE variable=b complete dim=0
	#pragma HLS ARRAY_RESHAPE variable=res complete dim=0

	#pragma HLS dataflow


	// Iterate over the rows of the A matrix
	for(int i = 0; i < MAT_A_ROWS; i++)
    {
		#pragma HLS unroll factor=4 skip_exit_check

		// Iterate over the columns of the B Matrix
		for(int j = 0; j < MAT_B_COLS; j++)
        {
				#pragma HLS unroll factor=4 skip_exit_check
				res[i][j] = 0;

				for(int k = 0; k < MAT_B_ROWS; k++)
                {
					#pragma HLS unroll factor=4 skip_exit_check

					res[i][j] += (result_t)(a[i][k]) * (result_t)(b[k][j]);
				}
		}
	}
}
