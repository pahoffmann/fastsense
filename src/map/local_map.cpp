
/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include "local_map.h"

#include <stdlib.h> // for abs
#include <stdexcept>

#include <util/logging/logger.h>

namespace fastsense::map
{

LocalMap::LocalMap(unsigned int sX, unsigned int sY, unsigned int sZ, const std::shared_ptr<GlobalMap>& map, const CommandQueuePtr& queue)
    : sizeX{sX % 2 == 1 ? sX : sX + 1},
      sizeY{sY % 2 == 1 ? sY : sY + 1},
      sizeZ{sZ % 2 == 1 ? sZ : sZ + 1},
      data{queue, sizeX * sizeY * sizeZ},
      posX{0},
      posY{0},
      posZ{0},
      offsetX{sizeX / 2},
      offsetY{sizeY / 2},
      offsetZ{sizeZ / 2},
      map{map}
{
    if (sX % 2 || sY % 2 || sZ % 2)
    {
        fastsense::util::logging::Logger::warning("Changed LocalMap size from even (", sX, ", ", sY, ", ", sZ, ") to odd (", sizeX, ", ", sizeY, ", ", sizeZ, ")");
    }

    for (size_t i = 0; i < sizeX * sizeY * sizeZ; i++)
    {
        data[i] = map->getValue(Vector3i(i / sizeZ / sizeY % sizeX - offsetX, i / sizeZ % sizeY - offsetY, i % sizeZ - offsetZ));
    }
}

LocalMap::~LocalMap()
{
}

std::pair<int, int>& LocalMap::value(int x, int y, int z)
{
    return value(Vector3i(x, y, z));
}

const std::pair<int, int>& LocalMap::value(int x, int y, int z) const
{
    return value(Vector3i(x, y, z));
}

std::pair<int, int>& LocalMap::value(const Vector3i& p)
{
    if (!inBounds(p))
    {
        throw std::out_of_range("Index out of bounds");
    }
    return data[((p.x() - posX + offsetX + sizeX) % sizeX) * sizeY * sizeZ +
                                              ((p.y() - posY + offsetY + sizeY) % sizeY) * sizeZ +
                                              (p.z() - posZ + offsetZ + sizeZ) % sizeZ];
}

const std::pair<int, int>& LocalMap::value(const Vector3i& p) const
{
    if (!inBounds(p))
    {
        throw std::out_of_range("Index out of bounds");
    }
    return data[((p.x() - posX + offsetX + sizeX) % sizeX) * sizeY * sizeZ +
                                              ((p.y() - posY + offsetY + sizeY) % sizeY) * sizeZ +
                                              (p.z() - posZ + offsetZ + sizeZ) % sizeZ];
}

void LocalMap::getSize(int* size) const
{
    size[0] = sizeX;
    size[1] = sizeY;
    size[2] = sizeZ;
}

void LocalMap::getPos(int* pos) const
{
    pos[0] = posX;
    pos[1] = posY;
    pos[2] = posZ;
}

bool LocalMap::inBounds(int x, int y, int z) const
{
    return abs(x - posX) <= sizeX / 2 && abs(y - posY) <= sizeY / 2 && abs(z - posZ) <= sizeZ / 2;
}

bool LocalMap::inBounds(const Vector3i& p) const
{
    return abs(p.x() - posX) <= sizeX / 2 && abs(p.y() - posY) <= sizeY / 2 && abs(p.z() - posZ) <= sizeZ / 2;
}

void LocalMap::shift(int x, int y, int z)
{
    // x
    int diffX = x - posX;
    for (int i = 0; i < abs(diffX); i++)
    {
        // step x
        for (int j = posY - sizeY / 2; j <= posY + sizeY / 2; j++)
        {
            for (int k = posZ - sizeZ / 2; k <= posZ + sizeZ / 2; k++)
            {
                if (diffX > 0)
                {
                    // step forward
                    std::pair<int, int>& inout = value(posX - sizeX / 2, j, k);
                    map->setValue(Vector3i(posX - sizeX / 2, j, k), inout);
                    inout = map->getValue(Vector3i(posX + sizeX / 2 + 1, j, k));
                }
                else
                {
                    // step backwards
                    std::pair<int, int>& inout = value(posX + sizeX / 2, j, k);
                    map->setValue(Vector3i(posX + sizeX / 2, j, k), inout);
                    inout = map->getValue(Vector3i(posX - sizeX / 2 - 1, j, k));
                }
            }
        }
        posX = diffX > 0 ? posX + 1 : posX - 1;
        offsetX = diffX > 0 ? (offsetX + 1) % sizeX : (offsetX - 1 + sizeX) % sizeX;
    }

    // y
    int diffY = y - posY;
    for (int i = 0; i < abs(diffY); i++)
    {
        // step y
        for (int j = posX - sizeX / 2; j <= posX + sizeX / 2; j++)
        {
            for (int k = posZ - sizeZ / 2; k <= posZ + sizeZ / 2; k++)
            {
                if (diffY > 0)
                {
                    // step forward
                    std::pair<int, int>& inout = value(j, posY - sizeY / 2, k);
                    map->setValue(Vector3i(j, posY - sizeY / 2, k), inout);
                    inout = map->getValue(Vector3i(j, posY + sizeY / 2 + 1, k));
                }
                else
                {
                    // step backwards
                    std::pair<int, int>& inout = value(j, posY + sizeY / 2, k);
                    map->setValue(Vector3i(j, posY + sizeY / 2, k), inout);
                    inout = map->getValue(Vector3i(j, posY - sizeY / 2 - 1, k));
                }
            }
        }
        posY = diffY > 0 ? posY + 1 : posY - 1;
        offsetY = diffY > 0 ? (offsetY + 1) % sizeY : (offsetY - 1 + sizeY) % sizeY;
    }

    // z
    int diffZ = z - posZ;
    for (int i = 0; i < abs(diffZ); i++)
    {
        // step z
        for (int j = posX - sizeX / 2; j <= posX + sizeX / 2; j++)
        {
            for (int k = posY - sizeY / 2; k <= posY + sizeY / 2; k++)
            {
                if (diffZ > 0)
                {
                    // step forward
                    std::pair<int, int>& inout = value(j, k, posZ - sizeZ / 2);
                    map->setValue(Vector3i(j, k, posZ - sizeZ / 2), inout);
                    inout = map->getValue(Vector3i(j, k, posZ + sizeZ / 2 + 1));
                }
                else
                {
                    // step backwards
                    std::pair<int, int>& inout = value(j, k, posZ + sizeZ / 2);
                    map->setValue(Vector3i(j, k, posZ + sizeZ / 2), inout);
                    inout = map->getValue(Vector3i(j, k, posZ - sizeZ / 2 - 1));
                }
            }
        }
        posZ = diffZ > 0 ? posZ + 1 : posZ - 1;
        offsetZ = diffZ > 0 ? (offsetZ + 1) % sizeZ : (offsetZ - 1 + sizeZ) % sizeZ;
    }
}

const buffer::InputOutputBuffer<std::pair<int, int>>& LocalMap::getBuffer() const
{
    return data;
}

LocalMapHW LocalMap::getHardwareRepresentation() const
{
    return {sizeX,
            sizeY,
            sizeZ,
            posX,
            posY,
            posZ,
            offsetX,
            offsetY,
            offsetZ};
}

} // namespace fastsense::map