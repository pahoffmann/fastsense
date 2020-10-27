
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

LocalMap::LocalMap(unsigned int sX, unsigned int sY, unsigned int sZ, const std::shared_ptr<GlobalMap> &map, const CommandQueuePtr &queue)
    : size_{static_cast<int>(sX % 2 == 1 ? sX : sX + 1),
            static_cast<int>(sY % 2 == 1 ? sY : sY + 1),
            static_cast<int>(sZ % 2 == 1 ? sZ : sZ + 1)},
      data_{queue, static_cast<size_t>(size_.x() * size_.y() * size_.z())},
      pos_{Vector3i::Zero()},
      offset_{size_ / 2},
      map_{map}
{
    if (sX % 2 || sY % 2 || sZ % 2)
    {
        fastsense::util::logging::Logger::warning("Changed LocalMap size from even (", sX, ", ", sY, ", ", sZ, ") to odd (", size_.x(), ", ", size_.y(), ", ", size_.z(), ")");
    }
    auto default_entry = map_->get_value(Vector3i(0, 0, 0));
    for (int i = 0; i < size_.x() * size_.y() * size_.z(); i++)
    {
        data_[i] = default_entry;
    }
}

std::pair<int, int> &LocalMap::value(int x, int y, int z)
{
    return value(Vector3i(x, y, z));
}

const std::pair<int, int> &LocalMap::value(int x, int y, int z) const
{
    return value(Vector3i(x, y, z));
}

std::pair<int, int> &LocalMap::value(Vector3i p)
{
    if (!in_bounds(p))
    {
        throw std::out_of_range("Index out of bounds");
    }
    p = p - pos_ + offset_ + size_;
    return data_[(p.x() % size_.x()) * size_.y() * size_.z() +
                 (p.y() % size_.y()) * size_.z() +
                  p.z() % size_.z()];
}

const std::pair<int, int> &LocalMap::value(Vector3i p) const
{
    if (!in_bounds(p))
    {
        throw std::out_of_range("Index out of bounds");
    }
    p = p - pos_ + offset_ + size_;
    return data_[(p.x() % size_.x()) * size_.y() * size_.z() +
                 (p.y() % size_.y()) * size_.z() +
                  p.z() % size_.z()];
}

const Vector3i &LocalMap::get_size() const
{
    return size_;
}

const Vector3i &LocalMap::get_pos() const
{
    return pos_;
}

const Vector3i &LocalMap::get_offset() const
{
    return offset_;
}

bool LocalMap::in_bounds(int x, int y, int z) const
{
    return in_bounds(Vector3i(x, y, z));
}

bool LocalMap::in_bounds(Vector3i p) const
{
    p = (p - pos_).cwiseAbs();
    return p.x() <= size_.x() / 2 && p.y() <= size_.y() / 2 && p.z() <= size_.z() / 2;
}

void LocalMap::shift(int x, int y, int z)
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
                    std::pair<int, int> &inout = value(pos_.x() - size_.x() / 2, j, k);
                    map_->set_value(Vector3i(pos_.x() - size_.x() / 2, j, k), inout);
                    inout = map_->get_value(Vector3i(pos_.x() + size_.x() / 2 + 1, j, k));
                }
                else
                {
                    // step backwards
                    std::pair<int, int> &inout = value(pos_.x() + size_.x() / 2, j, k);
                    map_->set_value(Vector3i(pos_.x() + size_.x() / 2, j, k), inout);
                    inout = map_->get_value(Vector3i(pos_.x() - size_.x() / 2 - 1, j, k));
                }
            }
        }
        pos_.x() = diffX > 0 ? pos_.x() + 1 : pos_.x() - 1;
        offset_.x() = diffX > 0 ? (offset_.x() + 1) % size_.x() : (offset_.x() - 1 + size_.x()) % size_.x();
    }

    // y
    int diffY = y - pos_.y();
    for (int i = 0; i < abs(diffY); i++)
    {
        // step y
        for (int j = pos_.x() - size_.x() / 2; j <= pos_.x() + size_.x() / 2; j++)
        {
            for (int k = pos_.z() - size_.z() / 2; k <= pos_.z() + size_.z() / 2; k++)
            {
                if (diffY > 0)
                {
                    // step forward
                    std::pair<int, int> &inout = value(j, pos_.y() - size_.y() / 2, k);
                    map_->set_value(Vector3i(j, pos_.y() - size_.y() / 2, k), inout);
                    inout = map_->get_value(Vector3i(j, pos_.y() + size_.y() / 2 + 1, k));
                }
                else
                {
                    // step backwards
                    std::pair<int, int> &inout = value(j, pos_.y() + size_.y() / 2, k);
                    map_->set_value(Vector3i(j, pos_.y() + size_.y() / 2, k), inout);
                    inout = map_->get_value(Vector3i(j, pos_.y() - size_.y() / 2 - 1, k));
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
                    std::pair<int, int> &inout = value(j, k, pos_.z() - size_.z() / 2);
                    map_->set_value(Vector3i(j, k, pos_.z() - size_.z() / 2), inout);
                    inout = map_->get_value(Vector3i(j, k, pos_.z() + size_.z() / 2 + 1));
                }
                else
                {
                    // step backwards
                    std::pair<int, int> &inout = value(j, k, pos_.z() + size_.z() / 2);
                    map_->set_value(Vector3i(j, k, pos_.z() + size_.z() / 2), inout);
                    inout = map_->get_value(Vector3i(j, k, pos_.z() - size_.z() / 2 - 1));
                }
            }
        }
        pos_.z() = diffZ > 0 ? pos_.z() + 1 : pos_.z() - 1;
        offset_.z() = diffZ > 0 ? (offset_.z() + 1) % size_.z() : (offset_.z() - 1 + size_.z()) % size_.z();
    }
}

buffer::InputOutputBuffer<std::pair<int, int>> &LocalMap::getBuffer()
{
    return data_;
}

LocalMapHW LocalMap::get_hardware_representation() const
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

void LocalMap::write_back()
{
    for (int i = pos_.x() - size_.x() / 2; i <= pos_.x() + size_.x() / 2; i++) 
    {
        for (int j = pos_.y() - size_.y() / 2; j <= pos_.y() + size_.y() / 2; j++) 
        {
            for (int k = pos_.z() - size_.z() / 2; k <= pos_.z() + size_.z() / 2; k++) 
            {
                std::pair<int, int> &out = value(i, j, k);
                map_->set_value(Vector3i(i, j, k), out);
            }
        }
    }

    map_->write_back();
}

} // namespace fastsense::map
