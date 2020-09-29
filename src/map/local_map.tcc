/**
 * @file ring_buffer.cpp
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#pragma once

#include <stdlib.h> // for abs
#include <stdexcept>
#include <util/logging/logger.h>

namespace fastsense::map
{

template<typename T>
LocalMap<T>::LocalMap(unsigned int sX, unsigned int sY, unsigned int sZ, const std::shared_ptr<GlobalMap>& map, const CommandQueuePtr& queue)
    : size_{(sX % 2 == 1 ? sX : sX + 1),
            (sY % 2 == 1 ? sY : sY + 1),
            (sZ % 2 == 1 ? sZ : sZ + 1)},
      data_{queue, size_.x() * size_.y() * size_.z()},
      pos_{0},
      offset_{size_.x() / 2, size_.y() / 2, size_.z() / 2},
      map_{map}
{
    if (sX % 2 || sY % 2 || sZ % 2)
    {
        fastsense::util::logging::Logger::warning("Changed LocalMap size from even (", sX, ", ", sY, ", ", sZ, ") to odd (", size_.x(), ", ", size_.y(), ", ", size_.z(), ")");
    }

    for (size_t i = 0; i < size_.x() * size_.y() * size_.z(); i++)
    {
        data_[i] = map_->getValue(i / size_.z() / size_.y() % size_.x() - offset_.x(), i / size_.z() % size_.y() - offset_.y(), i % size_.z() - offset_.z());
    }
}

template<typename T>
T& LocalMap<T>::value(int x, int y, int z)
{
    if (!inBounds(x, y, z))
    {
        throw std::out_of_range("Index out of bounds");
    }
    return data_[((x - pos_.x() + offset_.x() + size_.x()) % size_.x()) * size_.y() * size_.z() +
                                              ((y - pos_.y() + offset_.y() + size_.y()) % size_.y()) * size_.z() +
                                              (z - pos_.z() + offset_.z() + size_.z()) % size_.z()];
}

template<typename T>
const T& LocalMap<T>::value(int x, int y, int z) const
{
    if (!inBounds(x, y, z))
    {
        throw std::out_of_range("Index out of bounds");
    }
    return data_[((x - pos_.x() + offset_.x() + size_.x()) % size_.x()) * size_.y() * size_.z() +
                                              ((y - pos_.y() + offset_.y() + size_.y()) % size_.y()) * size_.z() +
                                              (z - pos_.z() + offset_.z() + size_.z()) % size_.z()];
}

template<typename T>
const util::LocalMapSize& LocalMap<T>::getSize() const
{
    return size_;
}

template<typename T>
const util::LocalMapPos& LocalMap<T>::getPos() const
{
    return pos_;
}

template<typename T>
const util::LocalMapOffset& LocalMap<T>::getOffset() const
{
    return offset_;
}

template<typename T>
bool LocalMap<T>::inBounds(int x, int y, int z) const
{
    return abs(x - pos_.x()) <= size_.x() / 2 && abs(y - pos_.y()) <= size_.y() / 2 && abs(z - pos_.z()) <= size_.z() / 2;
}

template<typename T>
void LocalMap<T>::shift(int x, int y, int z)
{
    // x
    int diffX = x - pos_.x();
    for (int i = 0; i < abs(diffX); i++)
    {
        // step x
        for (int j = pos_.y() - size_.y() / 2; j <= pos_.y() + size_.y() / 2; j++)
        {
            for (int k = pos_.z() - size_.z() / 2; k <= pos_.z() + size_.z() / 2; k++)
            {
                if (diffX > 0)
                {
                    // step forward
                    T& inout = value(pos_.x() - size_.x() / 2, j, k);
                    map_->setValue(pos_.x() - size_.x() / 2, j, k, inout);
                    inout = map_->getValue(pos_.x() + size_.x() / 2 + 1, j, k);
                }
                else
                {
                    // step backwards
                    T& inout = value(pos_.x() + size_.x() / 2, j, k);
                    map_->setValue(pos_.x() + size_.x() / 2, j, k, inout);
                    inout = map_->getValue(pos_.x() - size_.x() / 2 - 1, j, k);
                }
            }
        }
        pos_.x() = diffX > 0 ? pos_.x() + 1 : pos_.x() - 1;
        offset_.x() = diffX > 0 ? (offset_.x() + 1) % size_.x() : (offset_.x() - 1 + size_.x()) % size_.x();
    }

    // y
    int diffY = y - pos_.y();
    for (size_t i = 0; i < abs(diffY); i++)
    {
        // step y
        for (size_t j = pos_.x() - size_.x() / 2; j <= pos_.x() + size_.x() / 2; j++)
        {
            for (size_t k = pos_.z() - size_.z() / 2; k <= pos_.z() + size_.z() / 2; k++)
            {
                if (diffY > 0)
                {
                    // step forward
                    T& inout = value(j, pos_.y() - size_.y() / 2, k);
                    map_->setValue(j, pos_.y() - size_.y() / 2, k, inout);
                    inout = map_->getValue(j, pos_.y() + size_.y() / 2 + 1, k);
                }
                else
                {
                    // step backwards
                    T& inout = value(j, pos_.y() + size_.y() / 2, k);
                    map_->setValue(j, pos_.y() + size_.y() / 2, k, inout);
                    inout = map_->getValue(j, pos_.y() - size_.y() / 2 - 1, k);
                }
            }
        }
        pos_.y() = diffY > 0 ? pos_.y() + 1 : pos_.y() - 1;
        offset_.y() = diffY > 0 ? (offset_.y() + 1) % size_.y() : (offset_.y() - 1 + size_.y()) % size_.y();
    }

    // z
    int diffZ = z - pos_.z();
    for (int i = 0; i < abs(diffZ); i++)
    {
        // step z
        for (int j = pos_.x() - size_.x() / 2; j <= pos_.x() + size_.x() / 2; j++)
        {
            for (int k = pos_.y() - size_.y() / 2; k <= pos_.y() + size_.y() / 2; k++)
            {
                if (diffZ > 0)
                {
                    // step forward
                    T& inout = value(j, k, pos_.z() - size_.z() / 2);
                    map_->setValue(j, k, pos_.z() - size_.z() / 2, inout);
                    inout = map_->getValue(j, k, pos_.z() + size_.z() / 2 + 1);
                }
                else
                {
                    // step backwards
                    T& inout = value(j, k, pos_.z() + size_.z() / 2);
                    map_->setValue(j, k, pos_.z() + size_.z() / 2, inout);
                    inout = map_->getValue(j, k, pos_.z() - size_.z() / 2 - 1);
                }
            }
        }
        pos_.z() = diffZ > 0 ? pos_.z() + 1 : pos_.z() - 1;
        offset_.z() = diffZ > 0 ? (offset_.z() + 1) % size_.z() : (offset_.z() - 1 + size_.z()) % size_.z();
    }
}

template<typename T>
const buffer::InputOutputBuffer<T>& LocalMap<T>::getBuffer() const
{
    return data_;
}

template<typename T>
LocalMapHW LocalMap<T>::getHardwareRepresentation() const
{
    return {size_.x(),
            size_.y(),
            size_.z(),
            pos_.x(),
            pos_.y(),
            pos_.z(),
            offset_.x(),
            offset_.y(),
            offset_.z()};
}

}