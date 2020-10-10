#pragma once

/**
 * @author Marcel Flottmann
 */

#include <hls_math.h>

namespace fastsense
{

namespace map
{

template<typename T>
T overflow(T val, T max)
{
#pragma HLS INLINE
    return (val >= max) ? val - max : val;
    //return val % max;
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
        return hls::abs(x - posX) <= sizeX / 2 && hls::abs(y - posY) <= sizeY / 2 && hls::abs(z - posZ) <= sizeZ / 2;
    }

    int getIndex(int x, int y, int z) const
    {
#pragma HLS INLINE
        int x_offset = overflow(x - posX + offsetX + sizeX, sizeX) * sizeY * sizeZ;
        int y_offset = overflow(y - posY + offsetY + sizeY, sizeY) * sizeZ;
        int z_offset = overflow(z - posZ + offsetZ + sizeZ, sizeZ);
        return x_offset  + y_offset + z_offset;
    }

    template<typename T>
    T& get(T* data, int x, int y, int z) const
    {
#pragma HLS INLINE
        if (in_bounds(x, y, z))
        {
            return data[getIndex(x, y, z)];
        }
        return data[0];
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
