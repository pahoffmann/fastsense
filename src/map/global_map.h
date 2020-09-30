#pragma once

/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include <highfive/H5File.hpp>
#include <vector>
#include <cmath>
#include <string>
#include <utility>

#include <util/types.h>

// TODO handle existing/missing folder, where hdf5 will write

namespace fastsense::map
{

/**
 * Global map containing containing truncated signed distance function (tsdf) values and weights.
 * The map is divided into chunks.
 * An HDF5 file is used to store the chunks.
 * Additionally poses can be saved.
 */
class GlobalMap
{

private:

    /// Side length of the cube-shaped chunks. One chunk contains CHUNK_SIZE^3 * 2 entries (tsdf values and weights).
    const int CHUNK_SIZE = 16;
    const int SINGLE_SIZE = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;
    const int TOTAL_SIZE = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE * 2;

    /// Maximum number of active chunks.
    const int NUM_CHUNKS = 8;

    /**
     * HDF5 file in which the chunks are stored.
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
     * |
     * |-/poses
     * | |
     * | |-0 \
     * | |-1   pose datasets named in ascending order containing 6 values each
     * | |-2 /
     */
    HighFive::File file;

    /// Initial default tsdf value.
    int initialTsdfValue;

    /// Initial default weight.
    int initialWeight;

    /**
     * Vector of active chunks.
     * Each 3D chunk with tsdf values and weights is stored in a 1D vector.
     */
    std::vector<std::vector<int>> activeChunks;

    /**
     * Vector of the Positions of the active chunks.
     * Each entry in this vector corresponds to the entry in activeChunks with the same index.
     */
    std::vector<Vector3i> activeChunkPos;

    /**
     * Vector of the ages of the active chunks.
     * Each entry in this vector corresponds to the entry in activeChunks with the same index.
     * The age of a chunk counts, for how long the chunk has not been used.
     * It is used to determine which chunk gets replaced by a least recently used (LRU) strategy.
     */
    std::vector<int> ages;

    /// Number of poses that are saved in the HDF5 file
    int numPoses;

    /**
     * Given a position in a chunk the tag of the chunk gets returned.
     * @param pos the position
     * @return tag of the chunk
     */
    std::string tagFromChunkPos(const Vector3i& pos);

    /**
     * Returns the index of a global position in a chunk.
     * The returned index is that of the tsdf value.
     * The index of the weight is one greater.
     * @param pos the position
     * @return index in the chunk
     */
    int indexFromPos(Vector3i pos);

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
    std::vector<int>& activateChunk(const Vector3i& chunk);

public:

    /**
     * Constructor of the global map.
     * It is initialized without chunks.
     * The chunks are instead created dynamically depending on which are used.
     * @param name name with path and extension (.h5) of the HDF5 file in which the map is stored
     * @param initialTsdfValue initial default tsdf value
     * @param initialWeight initial default weight
     */
    GlobalMap(std::string name, int initialTsdfValue, int initialWeight);

    /**
     * Returns a value pair consisting of a tsdf value and a weight from the map.
     * @param pos the position
     * @return value pair from the map
     */
    std::pair<int, int> getValue(const Vector3i& pos);

    /**
     * Sets a value pair consisting of a tsdf value and a weight on the map.
     * @param pos the position
     * @param value value pair that is set
     */
    void setValue(const Vector3i& pos, std::pair<int, int> value);

    /**
     * Saves a pose in the HDF5 file.
     * @param x x-coordinate of the position of the pose
     * @param y y-coordinate of the position of the pose
     * @param z z-coordinate of the position of the pose
     * @param roll roll value of the rotation of the pose
     * @param pitch pitch value of the rotation of the pose
     * @param yaw yaw value of the rotation of the pose
     */
    void savePose(float x, float y, float z, float roll, float pitch, float yaw);

};

} // namespace fastsense::map