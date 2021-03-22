#pragma once

/**
 * @file local_map.h
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include <hw/buffer/buffer.h>
#include <map/local_map_hw.h>
#include "global_map.h"

namespace fastsense::map
{

/**
 * @brief Local Map that holds the part of the global map that is currently needed.
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
    buffer::InputOutputBuffer<TSDFValue> data_;

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
     * @brief Constructor of the local map.
     * 
     * The position is initialized to (0, 0, 0).
     * The sizes are given as parameters. If they are even, they are initialized as s + 1.
     * The offsets are initialized to size / 2, so that the ring boundarys match the array bounds.
     * 
     * @param sX Side length of the local map in the x direction
     * @param sY Side length of the local map in the y direction
     * @param sZ Side length of the local map in the z direction
     * @param map Pointer to the global map
     */
    LocalMap(unsigned int sX, unsigned int sY, unsigned int sZ, const std::shared_ptr<GlobalMap>& map, const CommandQueuePtr& queue);

    /**
     * @brief Destructor of the local map, deletes the array in particular.
     */
    ~LocalMap() = default;

    /**
     * @brief Copy constructor of the local map.
     * 
     * This constructor is needed in the asynchronous shift, update and visualization.
     * In the beginning of the thread the local map is copied
     * so that the the cloud callback and the map thread can work simultaneously.
     */
    LocalMap(const LocalMap&) = default;

    
    // Delete copy assignment operator of the local map.
    LocalMap& operator=(const LocalMap&) = delete;

    // Delete move assignment operator of the local map.
    LocalMap& operator=(LocalMap&&) = delete;

    // Delete move constructor of the local map.
    LocalMap(LocalMap&&) = delete;


    /**
     * @brief Swaps this local map with another one
     *
     * @param rhs the other map
     */
    void swap(LocalMap& rhs);

    /**
     * @brief Fills this local map with the data from another one
     *
     * @param rhs the other map
     */
    void fill_from(const LocalMap& rhs);

    /**
     * @brief Returns a value from the local map per reference.
     *        Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * 
     * @param x x-coordinate of the index in global coordinates
     * @param y y-coordinate of the index in global coordinates
     * @param z z-coordinate of the index in global coordinates
     * @return Value of the local map
     */
    inline TSDFValue& value(int x, int y, int z)
    {
        return value(Vector3i(x, y, z));
    }

    /**
     * @brief Returns a value from the local map per reference.
     *        Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * 
     * @param x x-coordinate of the index in global coordinates
     * @param y y-coordinate of the index in global coordinates
     * @param z z-coordinate of the index in global coordinates
     * @return Value of the local map
     */
    inline const TSDFValue& value(int x, int y, int z) const
    {
        return value(Vector3i(x, y, z));
    }

    /**
     * @brief Returns a value from the local map per reference.
     *        Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * 
     * @param p position of the index in global coordinates
     * @return value of the local map
     */
    inline TSDFValue& value(const Vector3i& p)
    {
        if (!in_bounds(p))
        {
            throw std::out_of_range("Index out of bounds");
        }
        return value_unchecked(p);
    }

    /**
     * @brief Returns a value from the local map per reference.
     *        Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * 
     * @param p position of the index in global coordinates
     * @return value of the local map
     */
    inline const TSDFValue& value(const Vector3i& p) const
    {
        if (!in_bounds(p))
        {
            throw std::out_of_range("Index out of bounds");
        }
        return value_unchecked(p);
    }

    /**
     * @brief Returns the size of the local map.
     * 
     * @return size of the local map
     */
    inline const Vector3i& get_size() const
    {
        return size_;
    }

    /**
     * @brief Returns the pos of the local map.
     * 
     * @return pos of the local map
     */
    inline const Vector3i& get_pos() const
    {
        return pos_;
    }

    /**
     * @brief Returns the offset of the local map.
     * 
     * @return offset of the local map
     */
    inline const Vector3i& get_offset() const
    {
        return offset_;
    }

    /**
     * @brief Shifts the local map, so that a new position is the center of the cuboid.
     * 
     * Entries, that stay in the buffer, stay in place.
     * Values outside of the buffer are loaded from and stored in the global map.
     * 
     * @param new_pos the new position. Must not be more than get_size() units away from get_pos()
     */
    void shift(const Vector3i& new_pos);

    /**
     * @brief Checks if x, y and z are within the current range.
     *
     * @param x x-coordinate to check
     * @param y y-coordinate to check
     * @param z z-coordinate to check
     * @return true if (x, y, z) is within the area of the buffer
     */
    inline bool in_bounds(int x, int y, int z) const
    {
        return in_bounds(Vector3i(x, y, z));
    }

    /**
     * @brief Checks if x-, y- and z-coordinates of the vector are within the current range.
     *
     * @param p position of the index in global coordinates
     * @return true if (x, y, z) is within the area of the buffer
     */
    inline bool in_bounds(Vector3i p) const
    {
        p = (p - pos_).cwiseAbs();
        return p.x() <= size_.x() / 2 && p.y() <= size_.y() / 2 && p.z() <= size_.z() / 2;
    }

    /**
     * @brief Returns the buffer in which the actual data of the local map is stored.
     *        It can be used with hardware kernels.
     * 
     * @return data buffer
     */
    buffer::InputOutputBuffer<TSDFValue>& getBuffer();

    LocalMapHW get_hardware_representation() const;

    /**
     * @brief Writes all data into the global map.
     *        Calls write_back of the global map to store the data in the file.
     */
    void write_back();

private:
    /**
     * @brief Writes to or reads from the global map in an area.
     *
     * @param bottom_corner the "bottom" corner of the area (smallest values along all axes); inclusive
     * @param top_corner the "top" corner of the area (biggest values along all axes); inclusive
     * @param save true: write to global map; false: read from global map
     */
    void save_load_area(const Vector3i& bottom_corner, const Vector3i& top_corner, bool save);

    /**
     * @brief Calculate the index of the given point.
     *
     * @param point the Point
     * @return int the index in data_
     */
    inline int get_index(const Vector3i& point) const
    {
        Vector3i p = point - pos_ + offset_ + size_;
        return (p.x() % size_.x()) * size_.y() * size_.z() +
               (p.y() % size_.y()) * size_.z() +
               p.z() % size_.z();
    }

    /**
     * @brief Returns a value from the local map per reference.
     *        In contrast to #value, it does not check the bounds.
     *
     * @param p position of the index in global coordinates
     * @return value of the local map
     */
    inline TSDFValue& value_unchecked(const Vector3i& p)
    {
        return data_[get_index(p)];
    }

    /**
     * @brief Returns a value from the local map per reference.
     *        In contrast to #value, it does not check the bounds.
     *
     * @param p position of the index in global coordinates
     * @return value of the local map
     */
    inline const TSDFValue& value_unchecked(const Vector3i& p) const
    {
        return data_[get_index(p)];
    }
};

} // namespace fastsense::map
