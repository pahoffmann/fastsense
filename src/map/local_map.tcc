#pragma once

/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include <stdlib.h> // for abs
#include <stdexcept>

#include <util/logging/logger.h>

namespace fastsense::map
{

template<typename T>
LocalMap<T>::LocalMap(unsigned int sX, unsigned int sY, unsigned int sZ, const std::shared_ptr<GlobalMap>& map, const CommandQueuePtr& queue)
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
        data[i] = map->getValue(i / sizeZ / sizeY % sizeX - offsetX, i / sizeZ % sizeY - offsetY, i % sizeZ - offsetZ);
    }
}

template<typename T>
LocalMap<T>::~LocalMap()
{
}

template<typename T>
T& LocalMap<T>::value(int x, int y, int z)
{
    if (!inBounds(x, y, z))
    {
        throw std::out_of_range("Index out of bounds");
    }
    return data[((x - posX + offsetX + sizeX) % sizeX) * sizeY * sizeZ +
                                              ((y - posY + offsetY + sizeY) % sizeY) * sizeZ +
                                              (z - posZ + offsetZ + sizeZ) % sizeZ];
}

template<typename T>
const T& LocalMap<T>::value(int x, int y, int z) const
{
    if (!inBounds(x, y, z))
    {
        throw std::out_of_range("Index out of bounds");
    }
    return data[((x - posX + offsetX + sizeX) % sizeX) * sizeY * sizeZ +
                                              ((y - posY + offsetY + sizeY) % sizeY) * sizeZ +
                                              (z - posZ + offsetZ + sizeZ) % sizeZ];
}

template<typename T>
void LocalMap<T>::getSize(int* size) const
{
    size[0] = sizeX;
    size[1] = sizeY;
    size[2] = sizeZ;
}

template<typename T>
void LocalMap<T>::getPos(int* pos) const
{
    pos[0] = posX;
    pos[1] = posY;
    pos[2] = posZ;
}

template<typename T>
bool LocalMap<T>::inBounds(int x, int y, int z) const
{
    return abs(x - posX) <= sizeX / 2 && abs(y - posY) <= sizeY / 2 && abs(z - posZ) <= sizeZ / 2;
}

template<typename T>
void LocalMap<T>::shift(int x, int y, int z)
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
                    T& inout = value(posX - sizeX / 2, j, k);
                    map->setValue(posX - sizeX / 2, j, k, inout);
                    inout = map->getValue(posX + sizeX / 2 + 1, j, k);
                }
                else
                {
                    // step backwards
                    T& inout = value(posX + sizeX / 2, j, k);
                    map->setValue(posX + sizeX / 2, j, k, inout);
                    inout = map->getValue(posX - sizeX / 2 - 1, j, k);
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
                    T& inout = value(j, posY - sizeY / 2, k);
                    map->setValue(j, posY - sizeY / 2, k, inout);
                    inout = map->getValue(j, posY + sizeY / 2 + 1, k);
                }
                else
                {
                    // step backwards
                    T& inout = value(j, posY + sizeY / 2, k);
                    map->setValue(j, posY + sizeY / 2, k, inout);
                    inout = map->getValue(j, posY - sizeY / 2 - 1, k);
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
                    T& inout = value(j, k, posZ - sizeZ / 2);
                    map->setValue(j, k, posZ - sizeZ / 2, inout);
                    inout = map->getValue(j, k, posZ + sizeZ / 2 + 1);
                }
                else
                {
                    // step backwards
                    T& inout = value(j, k, posZ + sizeZ / 2);
                    map->setValue(j, k, posZ + sizeZ / 2, inout);
                    inout = map->getValue(j, k, posZ - sizeZ / 2 - 1);
                }
            }
        }
        posZ = diffZ > 0 ? posZ + 1 : posZ - 1;
        offsetZ = diffZ > 0 ? (offsetZ + 1) % sizeZ : (offsetZ - 1 + sizeZ) % sizeZ;
    }
}

template<typename T>
const buffer::InputOutputBuffer<T>& LocalMap<T>::getBuffer() const
{
    return data;
}

template<typename T>
LocalMapHW LocalMap<T>::getHardwareRepresentation() const
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