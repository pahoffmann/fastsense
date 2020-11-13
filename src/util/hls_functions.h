#pragma once

/**
 * @file hls_functions.h
 * @author Malte Hillmann
 */

//#include <hls_math.h>
#ifdef __SYNTHESIS__
#include <hls_math.h>
#else
#include <math.h>
#endif

inline int hls_abs(int x)
{
    return x < 0 ? -x : x;
}

inline int hls_sqrt_approx(int x)
{
#ifdef __SYNTHESIS__
    return hls::sqrt(x);
#else
    return std::round(std::sqrt(x));
#endif
}

inline float hls_sqrt_float(float x)
{
#ifdef __SYNTHESIS__
    return hls::sqrt(x);
#else
    return std::sqrt(x);
#endif
}

inline void hls_sincos(float angle, float* sin_out, float* cos_out)
{
#ifdef __SYNTHESIS__
    return hls::sincos(angle, &sin_out, &cos_out);
#else
    sincosf(angle, sin_out, cos_out);
#endif
}
