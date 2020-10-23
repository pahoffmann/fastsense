#pragma once

/**
 * @file hls_functions.h
 * @author Malte Hillmann
 */

#include <hls_math.h>
/*#ifdef __SYNTHESIS__
#include <hls_math.h>
#else
#include <cmath>
#endif*/

inline int hls_abs(int x)
{
    return x < 0 ? -x : x;
}

inline int hls_sqrt_approx(int x)
{
    return hls::sqrt(x);
/*#ifdef __SYNTHESIS__
    return hls::sqrt(x);
#else
    // return std::sqrt(x);
    return std::round(std::sqrt(x));
#endif*/
    // // meh, doesn't work
    // if (x <= 0)
    // {
    //     return 0;
    // }
    // int a = 0, b = x;
    // while (b - a > 1)
    // {
    //     int mid = (a + b) / 2;
    //     int mm = mid * mid;
    //     if (mm < x)
    //     {
    //         a = mid;
    //     }
    //     else if (mm == x)
    //     {
    //         return mid;
    //     }
    //     else
    //     {
    //         b = mid;
    //     }
    // }
    // return a;
}
