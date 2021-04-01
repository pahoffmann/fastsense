#pragma once

/**
 * @file global_map.h
 * @author Steffen Hinderink
 * @author Juri Vana
 * @author Malte Hillmann
 */

#include <highfive/H5File.hpp>
#include <cmath>
#include <string>
#include <utility>
#include <util/point.h>
#include <util/tsdf.h>

namespace fastsense::map
{

struct ActiveChunk
{
    std::vector<TSDFEntry::RawType> data;
    Vector3i pos;
    int age;
};

/**
 * Global map containing containing truncated signed distance function (tsdf) values and weights.
 * The map is divided into chunks.
 * An HDF5 file is used to store the chunks.
 * Additionally poses can be saved.
 */
class GlobalMap
{

private:
 /**
     * Saves a pose in the HDF5 file.
     * @param t_x x-coordinate of the position of the pose
     * @param t_y y-coordinate of the position of the pose
     * @param t_z z-coordinate of the position of the pose
     * @param quat_x x-value of the rotation quaternion of the pose
     * @param quat_y y-value of the rotation quaternion of the pose
     * @param quat_z z-value of the rotation quaternion of the pose
     * @param quat_w w-value of the rotation quaternion of the pose
     */
    void save_pose(float t_x, float t_y, float t_z, float quat_x, float quat_y, float quat_z, float quat_w);

     * The file structure looks like this:
     *
     * file.h5
     * |
     * |-/map
     * | |
     * | |-0_0_0 \
     * | |-0_0_1  \
     * | |-0_1_0    chunk datasets named after their tag
     * | |-0_1_1  /
     * | |-1_0_0 /
     */
    HighFive::File file_;
 /**
     * Saves a pose in the HDF5 file.
     * @param t_x x-coordinate of the position of the pose
     * @param t_y y-coordinate of the position of the pose
     * @param t_z z-coordinate of the position of the pose
     * @param quat_x x-value of the rotation quaternion of the pose
     * @param quat_y y-value of the rotation quaternion of the pose
     * @param quat_z z-value of the rotation quaternion of the pose
     * @param quat_w w-value of the rotation quaternion of the pose
     */
    void save_pose(float t_x, float t_y, float t_z, float quat_x, float quat_y, float quat_z, float quat_w);

     * Given a position in a chunk the tag of the chunk gets returned.
     * @param pos the position
     * @return tag of the chunk
     */
    std::string tag_from_chunk_pos(const Vector3i& pos);

    /**
     * Returns the index of a global position in a chunk.
     * The returned index is that of the tsdf value.
     * The index of the weight is one greater.
     * @param pos the position
     * @return index in the chunk
     */
    int index_from_pos(Vector3i pos, const Vector3i& chunkPos);

public:

    /// Side length of the cube-shaped chunks
    static constexpr int CHUNK_SIZE = 64;

    /// Maximum number of active chunks.
    static constexpr int NUM_CHUNKS = 64;

    /**
     * Constructor of the global map.
     * It is initialized without chunks.
     * The chunks are instead created dynamically depending on which are used.
     * @param name name with path and extension (.h5) of the HDF5 file in which the map is stored
     * @param initial_tsdf_value default tsdf value
     * @param initial_weight initial default weight
     */
    GlobalMap(std::string name, TSDFEntry::ValueType initial_tsdf_value, TSDFEntry::WeightType initial_weight);

    /**
     * Returns a value pair consisting of a tsdf value and a weight from the map.
     * @param pos the position
     * @return value pair from the map
     */
    TSDFEntry get_value(const Vector3i& pos);

    /**
     * Sets a value pair consisting of a tsdf value and a weight on the map.
     * @param pos the position
     * @param value value pair that is set
     */
    void set_value(const Vector3i& pos, const TSDFEntry& value);

    /**
     * Activates a chunk and returns it by reference.
     * If the chunk was already active, it is simply returned.
     * Else the HDF5 file is checked for the chunk.
     * If it also doesn't exist there, a new empty chunk is created.
     * Chunks get replaced and written into the HDF5 file by a LRU strategy.
     * The age of the activated chunk is reset and all ages are updated.
     * @param chunk position of the chunk that gets activated
     * @return reference to the activated chunk
     */
    std::vector<TSDFEntry::RawType>& activate_chunk(const Vector3i& chunk);

    /**
     * Writes all active chunks into the HDF5 file.
     */
    void write_back();

};

} // namespace fastsense::map
