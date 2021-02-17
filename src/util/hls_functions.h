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

template<typename T>
inline T hls_abs(T x)
{
    return x < 0 ? T(-x) : T(x);
}

inline long hls_abs_arith(long x)
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

inline long hls_sqrt_approx_arith(long x)
{
    return std::round(std::sqrt(x));
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
