#pragma once

/**
 * @author Marc Eisoldt
 */

#include <cmath>

int norm(int* point)
{
#pragma HLS inline
	return std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
}
