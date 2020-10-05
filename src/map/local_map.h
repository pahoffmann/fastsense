#pragma once

/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include <hw/buffer/buffer.h>
#include <map/local_map_hw.h>
#include "global_map.h"
#include <util/types.h>

namespace fastsense::map
{

/**
 * Three dimensional array that can be shifted without needing to copy every entry.
 * This is done by implementing it as a ring in every dimension.
 * It is used to store truncated signed distance function (tsdf) values.
 */
class LocalMap
{

private:

    /**
     * Side lengths of the local map. They are always odd, so that there is a central cell.
     * The local map contains x * y * z values.
     */
    Vector3i size_;

    /// Actual data of the local map.
    buffer::InputOutputBuffer<std::pair<int, int>> data_;

    /// Position (x, y, z) of the center of the cuboid in global coordinates.
    Vector3i pos_;

    /**
     * Offset (x, y, z) of the data in the ring.
     * Each variable is the index of the center of the cuboid in the data array in its dimension.
     *
     * If size = 5, pos = 1 (-> indices range from -1 to 3) and offset = 3 (in one dimension),
     * the indices of the global map in the data array are for example:
     * 3 -1  0  1  2
     *          ^
     *       offset
     */
    Vector3i offset_;

    /// Pointer to the global map in which the values outside of the buffer are stored
    std::shared_ptr<GlobalMap> map_;

public:

    /**
     * Constructor of the local map.
     * The position is initialized to (0, 0, 0).
     * The sizes are given as parameters. If they are even, they are initialized as s + 1.
     * The offsets are initialized to size / 2, so that the ring boundarys match the array bounds.
     * @param sX Side length of the local map in the x direction
     * @param sY Side length of the local map in the y direction
     * @param sZ Side length of the local map in the z direction
     * @param map Pointer to the global map
     */
    LocalMap(unsigned int sX, unsigned int sY, unsigned int sZ, const std::shared_ptr<GlobalMap>& map, const CommandQueuePtr& queue);

    /**
     * Destructor of the local map.
     * Deletes the array in particular.
     */
    virtual ~LocalMap() = default;

    /**
     * Returns a value from the local map per reference.
     * Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * @param x x-coordinate of the index in global coordinates
     * @param y y-coordinate of the index in global coordinates
     * @param z z-coordinate of the index in global coordinates
     * @return Value of the local map
     */
    std::pair<int, int>& value(int x, int y, int z);

    /**
     * Returns a value from the local map per reference.
     * Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * @param x x-coordinate of the index in global coordinates
     * @param y y-coordinate of the index in global coordinates
     * @param z z-coordinate of the index in global coordinates
     * @return Value of the local map
     */
    const std::pair<int, int>& value(int x, int y, int z) const;

    /**
     * Returns a value from the local map per reference.
     * Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * @param p position of the index in global coordinates
     * @return value of the local map
     */
    std::pair<int, int>& value(Vector3i p);

    /**
     * Returns a value from the local map per reference.
     * Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * @param p position of the index in global coordinates
     * @return value of the local map
     */
    const std::pair<int, int>& value(Vector3i p) const;

    /**
     * Returns the size of the local map
     * @return size of the local map
     */
    const Vector3i& get_size() const;

    /**
     * Returns the pos of the local map
     * @return pos of the local map
     */
    const Vector3i& get_pos() const;

    /**
     * Returns the offset of the local map
     * @return offset of the local map
     */
    const Vector3i& get_offset() const;

    /**
     * Shifts the local map, so that a new position is the center of the cuboid.
     * Entries, that stay in the buffer, stay in place.
     * Values outside of the buffer are loaded from and stored in the global map.
     * @param x x-coordinate of the new position
     * @param y y-coordinate of the new position
     * @param z z-coordinate of the new position
     */
    void shift(int x, int y, int z);

    /**
     * Checks if x, y and z are within the current range
     *
     * @param x x-coordinate to check
     * @param y y-coordinate to check
     * @param z z-coordinate to check
     * @return true if (x, y, z) is within the area of the buffer
     */
    bool in_bounds(int x, int y, int z) const;

    /**
     * Checks if x, y and z are within the current range
     *
     * @param p position of the index in global coordinates
     * @return true if (x, y, z) is within the area of the buffer
     */
    bool in_bounds(Vector3i p) const;

    /**
     * Returns the buffer in which the actual data of the local map is stored.
     * It can be used with hardware kernels.
     * @return data buffer
     */
    const buffer::InputOutputBuffer<std::pair<int, int>>& getBuffer() const;

    LocalMapHW get_hardware_representation() const;

    /**
     * Writes all data into the global map.
     * Calls write_back of the global map to store the data in the file.
     */
    void write_back();
};

} // namespace fastsense::map
