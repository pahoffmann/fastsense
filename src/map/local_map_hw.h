#pragma once

/**
 * @file local_map_hw.h
 * @author Marcel Flottmann
 */

#include <util/hls_functions.h>
#include <iostream>

namespace fastsense
{

namespace map
{

template<typename T>
T overflow(T val, T max)
{
#pragma HLS INLINE
    //return (val >= max) ? val - max : val;
    return val % max;
}

/**
 * @brief Data Transfer Object of LocalMap for hardware with hardware optimized access functions
 *
 */
struct LocalMapHW
{
    int sizeX;
    int sizeY;
    int sizeZ;
    int posX;
    int posY;
    int posZ;
    int offsetX;
    int offsetY;
    int offsetZ;

    bool in_bounds(int x, int y, int z) const
    {
#pragma HLS INLINE
        return hls_abs(x - posX) <= sizeX / 2 && hls_abs(y - posY) <= sizeY / 2 && hls_abs(z - posZ) <= sizeZ / 2;
    }

    int getIndex(int x, int y, int z) const
    {
#pragma HLS INLINE
        int x_offset = overflow(x - posX + offsetX + sizeX, sizeX) * sizeY * sizeZ;
        int y_offset = overflow(y - posY + offsetY + sizeY, sizeY) * sizeZ;
        int z_offset = overflow(z - posZ + offsetZ + sizeZ, sizeZ);
        int index = x_offset  + y_offset + z_offset;
        return index;
    }

    template<typename T>
    T get(T* data, int x, int y, int z) const
    {
#pragma HLS INLINE
        if (in_bounds(x, y, z))
        {
            return data[getIndex(x, y, z)];
        }

        return T();
    }

    template<typename T>
    void set(T* data, int x, int y, int z, const T& val) const
    {
#pragma HLS INLINE
        if (in_bounds(x, y, z))
        {
            data[getIndex(x, y, z)] = val;
        }
    }
};

} // namespace map

} // namespace fastsense
