/**
 * @author Steffen Hinderink
 * @author Juri Vana
 * @author Malte Hillmann
 */

#include <sstream>

#include "global_map.h"

using namespace fastsense::map;

GlobalMap::GlobalMap(std::string name, int initialTsdfValue, int initialWeight)
    : file{name, HighFive::File::OpenOrCreate | HighFive::File::Truncate}, // Truncate clears already existing file
      initialTsdfValue{initialTsdfValue},
      initialWeight{initialWeight},
      activeChunks{},
      numPoses{0}
{
    if (!file.exist("/map"))
    {
        file.createGroup("/map");
    }
    if (!file.exist("/poses"))
    {
        file.createGroup("/poses");
    }
}

std::string GlobalMap::tagFromChunkPos(const Vector3i& pos)
{
    std::stringstream ss;
    ss << pos.x() << "_" << pos.y() << "_" << pos.z();
    return ss.str();
}
int GlobalMap::indexFromPos(Vector3i pos, const Vector3i& chunkPos)
{
    pos -= chunkPos * CHUNK_SIZE;
    return (pos.x() * CHUNK_SIZE * CHUNK_SIZE + pos.y() * CHUNK_SIZE + pos.z()) * 2;
}

std::vector<int>& GlobalMap::activateChunk(const Vector3i& chunkPos)
{
    int index = -1;
    int n = activeChunks.size(); // == ages.size() == activeChunkPos.size()
    for (int i = 0; i < n; i++)
    {
        if (activeChunks[i].pos == chunkPos)
        {
            // chunk is already active
            index = i;
        }
    }
    if (index == -1)
    {
        // chunk is not already active
        ActiveChunk newChunk;
        newChunk.pos = chunkPos;
        newChunk.age = 0;

        HighFive::Group g = file.getGroup("/map");
        auto tag = tagFromChunkPos(chunkPos);
        if (g.exist(tag))
        {
            // read chunk from file
            HighFive::DataSet d = g.getDataSet(tag);
            d.read(newChunk.chunk);
        }
        else
        {
            // create new chunk
            newChunk.chunk = std::vector<int>(TOTAL_SIZE);
            for (int i = 0; i < SINGLE_SIZE; i++)
            {
                newChunk.chunk[2 * i] = initialTsdfValue;
                newChunk.chunk[2 * i + 1] = initialWeight;
            }
        }
        // put new chunk into activeChunks
        if (n < NUM_CHUNKS)
        {
            // there is still room for active chunks
            index = n;
            newChunk.age = n; // temporarily assign oldest age so that all other ages get incremented
            activeChunks.push_back(newChunk);
        }
        else
        {
            // write oldest chunk into file
            int max = -1;
            for (int i = 0; i < n; i++)
            {
                if (activeChunks[i].age > max)
                {
                    max = activeChunks[i].age;
                    index = i;
                }
            }
            auto tag = tagFromChunkPos(activeChunks[index].pos);

            if(tag == "-1_0_0")
            {
                std::cout << "test" << std::endl;
            }

            if (g.exist(tag))
            {
                auto d = g.getDataSet(tag);
                d.write(activeChunks[index].chunk);
            }
            else
            {
                g.createDataSet(tag, activeChunks[index].chunk);
            }
            // overwrite with new chunk
            activeChunks[index] = newChunk;
        }
    }
    // update ages
    int age = activeChunks[index].age;
    for (auto& chunk : activeChunks)
    {
        if (chunk.age < age)
        {
            chunk.age++;
        }
    }
    activeChunks[index].age = 0;
    return activeChunks[index].chunk;
}

std::pair<int, int> GlobalMap::getValue(const Vector3i& pos)
{
    Vector3i chunkPos = floor_shift(pos, CHUNK_SHIFT);
    const auto& chunk = activateChunk(chunkPos);
    int index = indexFromPos(pos, chunkPos);
    return std::make_pair(chunk[index], chunk[index + 1]);
}

void GlobalMap::setValue(const Vector3i& pos, const std::pair<int, int>& value)
{
    Vector3i chunkPos = floor_shift(pos, CHUNK_SHIFT);
    auto& chunk = activateChunk(chunkPos);
    int index = indexFromPos(pos, chunkPos);
    chunk[index] = value.first;
    chunk[index + 1] = value.second;
}

void GlobalMap::savePose(float x, float y, float z, float roll, float pitch, float yaw)
{
    HighFive::Group g = file.getGroup("/poses");
    std::vector<float> pose {x, y, z, roll, pitch, yaw};
    std::stringstream ss;
    ss << numPoses;
    g.createDataSet(ss.str(), pose);
    numPoses++;
}
