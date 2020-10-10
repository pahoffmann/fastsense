#pragma once

/**
 * @author Marc Eisoldt
 */

#include <hls_math.h>

int norm(int* point)
{
#pragma HLS inline
	return hls::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
}
