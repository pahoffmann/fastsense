#pragma once

/**
 * @file local_map_hw.h
 * @author Marcel Flottmann
 */

#include <util/hls_functions.h>
#include <util/tsdf_hw.h>
#include <iostream>

namespace fastsense
{

namespace map
{

/**
 * @brief Modulo operation with values up to the double the max value for hardware imlpementation
 * 
 * @tparam T 
 * @param val Value for the calculation
 * @param max Maximum value
 * @return T 
 */
template<typename T>
T overflow(T val, T max)
{
#pragma HLS INLINE
    if (val >= 2 * max)
    {
        return val - 2 * max;
    }
    else if (val >= max)
    {
        return val - max;
    }
    else
    {
        return val;
    }
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

    /**
     * @brief Checks if the coordinates are inside the local map
     * 
     * @param x X component in map coordinates
     * @param y Y component in map coordinates
     * @param z Z component in map coordinates
     * @return true Coordiantes are inside the local map
     * @return false Coordiantes are outside the local map
     */
    bool in_bounds(int x, int y, int z) const
    {
#pragma HLS INLINE
        return hls_abs(x - posX) <= sizeX / 2 && hls_abs(y - posY) <= sizeY / 2 && hls_abs(z - posZ) <= sizeZ / 2;
    }

    /**
     * @brief Calculate the index for the local map buffer from map coordinates
     * 
     * @param x X component in map coordinates
     * @param y Y component in map coordinates
     * @param z Z component in map coordinates
     * @return int Index for the local map buffer
     */
    int getIndex(int x, int y, int z) const
    {
#pragma HLS INLINE
        int x_offset = overflow(x - posX + offsetX + sizeX, sizeX) * sizeY * sizeZ;
        int y_offset = overflow(y - posY + offsetY + sizeY, sizeY) * sizeZ;
        int z_offset = overflow(z - posZ + offsetZ + sizeZ, sizeZ);
        int index = x_offset  + y_offset + z_offset;
        return index;
    }

    /**
     * @brief Load the TSDF value at the given map coordinates. If out of bounds, a default value is returned (no error)!
     * 
     * @param data Local map buffer
     * @param x X component in map coordinates
     * @param y Y component in map coordinates
     * @param z Z component in map coordinates
     * @return TSDFValueHW TSDFValue at the coordinates
     */
    TSDFValueHW get(TSDFValueHW* data, int x, int y, int z) const
    {
#pragma HLS INLINE
        if (in_bounds(x, y, z))
        {
            return data[getIndex(x, y, z)];
        }

        return TSDFValueHW{0, 0};
    }

    /**
     * @brief Set the TSDF value at the given map coordinates
     * 
     * @param data Local map buffer
     * @param x X component in map coordinates
     * @param y Y component in map coordinates
     * @param z Z component in map coordinates
     * @param val New TSDF value
     */
    void set(TSDFValueHW* data, int x, int y, int z, const TSDFValueHW& val) const
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
