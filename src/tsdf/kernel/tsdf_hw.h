#pragma once

/**
 * @author Marc Eisoldt
 */

#define NO_SYNTH

#ifdef NO_SYNTH
	#include <cmath>
#else
	#include <hls_math.h>
#endif

int norm(int* point)
{
#pragma HLS inline

	#ifdef NO_SYNTH
		return std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
	#else
		return hls::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
	#endif
}
