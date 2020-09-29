/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include "global_map.h"

using namespace fastsense::map;

GlobalMap::GlobalMap(std::string name, float initialTsdfValue, float initialWeight)
    : file{name, HighFive::File::OpenOrCreate | HighFive::File::Truncate}, // Truncate clears already existing file
      initialTsdfValue{initialTsdfValue},
      initialWeight{initialWeight},
      activeChunks{},
      tags{},
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

std::string GlobalMap::tagFromPos(int x, int y, int z)
{
    std::stringstream ss;
    ss << std::floor((float) x / CHUNK_SIZE) << "_" << std::floor((float) y / CHUNK_SIZE) << "_" << std::floor((float) z / CHUNK_SIZE);
    return ss.str();
}

int GlobalMap::indexFromPos(int x, int y, int z)
{
    x -= std::floor((float) x / CHUNK_SIZE) * CHUNK_SIZE;
    y -= std::floor((float) y / CHUNK_SIZE) * CHUNK_SIZE;
    z -= std::floor((float) z / CHUNK_SIZE) * CHUNK_SIZE;
    return (x * CHUNK_SIZE * CHUNK_SIZE + y * CHUNK_SIZE + z) * 2;
}

std::vector<float>& GlobalMap::activateChunk(std::string tag)
{
    int index = -1;
    int n = activeChunks.size(); // = ages.size() = tags.size()
    for (int i = 0; i < n; i++)
    {
        if (tags[i] == tag)
        {
            // chunk is already active
            index = i;
        }
    }
    if (index == -1)
    {
        // chunk is not already active
        std::vector<float> newChunk;
        HighFive::Group g = file.getGroup("/map");
        if (g.exist(tag))
        {
            // read chunk from file
            HighFive::DataSet d = g.getDataSet(tag);
            d.read(newChunk);
        }
        else
        {
            // create new chunk
            newChunk = std::vector<float>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE * 2);
            for (int i = 0; i < CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE; i++)
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
            tags.push_back(tag);
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
            if (g.exist(tags[index]))
            {
                auto d = g.getDataSet(tags[index]);
                d.write(activeChunks[index]);
            }
            else
            {
                g.createDataSet(tags[index], activeChunks[index]);
            }
            // overwrite with new chunk
            activeChunks[index] = newChunk;
            tags[index] = tag;
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

std::pair<float, float> GlobalMap::getValue(int x, int y, int z)
{
    std::vector<float>& chunk = activateChunk(tagFromPos(x, y, z));
    int index = indexFromPos(x, y, z);
    return std::pair<float, float>(chunk[index], chunk[index + 1]);
}

void GlobalMap::setValue(int x, int y, int z, std::pair<float, float> value)
{
    std::vector<float>& chunk = activateChunk(tagFromPos(x, y, z));
    int index = indexFromPos(x, y, z);
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
