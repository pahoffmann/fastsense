/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include <sstream>

#include "global_map.h"

using namespace fastsense::map;

GlobalMap::GlobalMap(std::string name, int initialTsdfValue, int initialWeight)
    : file{name, HighFive::File::OpenOrCreate | HighFive::File::Truncate}, // Truncate clears already existing file
      initialTsdfValue{initialTsdfValue},
      initialWeight{initialWeight},
      activeChunks{},
      activeChunkPos{},
      ages{},
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
int GlobalMap::indexFromPos(Vector3i pos)
{
    pos -= (pos / CHUNK_SIZE) * CHUNK_SIZE;
    return (pos.x() * CHUNK_SIZE * CHUNK_SIZE + pos.y() * CHUNK_SIZE + pos.z()) * 2;
}

std::vector<int>& GlobalMap::activateChunk(const Vector3i& chunkPos)
{
    int index = -1;
    int n = activeChunks.size(); // == ages.size() == activeChunkPos.size()
    for (int i = 0; i < n; i++)
    {
        if (activeChunkPos[i] == chunkPos)
        {
            // chunk is already active
            index = i;
        }
    }
    if (index == -1)
    {
        // chunk is not already active
        std::vector<int> newChunk;
        HighFive::Group g = file.getGroup("/map");
        auto tag = tagFromChunkPos(chunkPos);
        if (g.exist(tag))
        {
            // read chunk from file
            HighFive::DataSet d = g.getDataSet(tag);
            d.read(newChunk);
        }
        else
        {
            // create new chunk
            newChunk = std::vector<int>(TOTAL_SIZE);
            for (int i = 0; i < SINGLE_SIZE; i++)
            {
                newChunk[2 * i] = initialTsdfValue;
                newChunk[2 * i + 1] = initialWeight;
            }
        }
        // put new chunk into activeChunks
        if (n < NUM_CHUNKS)
        {
            // there is still room for active chunks
            index = n;
            activeChunks.push_back(newChunk);
            activeChunkPos.push_back(chunkPos);
            ages.push_back(n); // temporarily assign oldest age so that all other ages get incremented
        }
        else
        {
            // write oldest chunk into file
            int max = -1;
            for (int i = 0; i < n; i++)
            {
                if (ages[i] > max)
                {
                    max = ages[i];
                    index = i;
                }
            }
            auto tag = tagFromChunkPos(activeChunkPos[index]);
            if (g.exist(tag))
            {
                auto d = g.getDataSet(tag);
                d.write(activeChunks[index]);
            }
            else
            {
                g.createDataSet(tag, activeChunks[index]);
            }
            // overwrite with new chunk
            activeChunks[index] = newChunk;
            activeChunkPos[index] = chunkPos;
        }
    }
    // update ages
    int age = ages[index];
    for (size_t i = 0; i < ages.size(); i++)
    {
        if (ages[i] < age)
        {
            ages[i]++;
        }
    }
    ages[index] = 0;
    return activeChunks[index];
}

std::pair<int, int> GlobalMap::getValue(const Vector3i& pos)
{
    const auto& chunk = activateChunk(pos / CHUNK_SIZE);
    int index = indexFromPos(pos);
    return std::pair<int, int>(chunk[index], chunk[index + 1]);
}

void GlobalMap::setValue(const Vector3i& pos, std::pair<int, int> value)
{
    std::vector<int>& chunk = activateChunk(pos / CHUNK_SIZE);
    int index = indexFromPos(pos);
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
