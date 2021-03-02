
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
    : size_{static_cast<int>(sX % 2 == 1 ? sX : sX + 1),
            static_cast<int>(sY % 2 == 1 ? sY : sY + 1),
            static_cast<int>(sZ % 2 == 1 ? sZ : sZ + 1)},
      data_{queue, static_cast<size_t>(size_.x() * size_.y() * size_.z())},
      pos_{Vector3i::Zero()},
      offset_{size_ / 2},
      map_{map}
{
    if (sX % 2 == 0 || sY % 2 == 0 || sZ % 2 == 0)
    {
        fastsense::util::logging::Logger::warning("Changed LocalMap size from even (", sX, ", ", sY, ", ", sZ, ") to odd (", size_.x(), ", ", size_.y(), ", ", size_.z(), ")");
    }
    auto default_entry = map_->get_value(Vector3i(0, 0, 0));
    for (int i = 0; i < size_.x() * size_.y() * size_.z(); i++)
    {
        data_[i] = default_entry;
    }
}

void LocalMap::swap(LocalMap& rhs)
{
    this->data_.swap(rhs.data_);
    std::swap(this->size_, rhs.size_);
    std::swap(this->pos_, rhs.pos_);
    std::swap(this->offset_, rhs.offset_);
    std::swap(this->map_, rhs.map_);
}

void LocalMap::fill_from(const LocalMap& rhs)
{
    this->data_.fill_from(rhs.data_);
    this->size_ = rhs.size_;
    this->pos_ = rhs.pos_;
    this->offset_ = rhs.offset_;
    this->map_ = rhs.map_;
}

void LocalMap::shift(const Vector3i& new_pos)
{
    Vector3i diff = new_pos - pos_;
    for (int axis = 0; axis < 3; axis++)
    {
        if (diff[axis] == 0)
        {
            continue;
        }
        if (std::abs(diff[axis]) > size_[axis])
        {
            throw std::invalid_argument("Map Shift is larger than Map");
        }
        Vector3i start = pos_ - size_ / 2;
        Vector3i end = pos_ + size_ / 2;
        if (diff[axis] > 0)
        {
            // -1 because save_load_area takes inclusive ranges
            end[axis] = start[axis] + diff[axis] - 1;
        }
        else
        {
            // diff is negative and ranges are inclusive: end - (std::(diff) - 1)
            start[axis] = end[axis] + diff[axis] + 1;
        }
        save_load_area(start, end, true);
        pos_[axis] += diff[axis];
        offset_[axis] = (offset_[axis] + diff[axis] + size_[axis]) % size_[axis];

        start = pos_ - size_ / 2;
        end = pos_ + size_ / 2;
        if (diff[axis] > 0)
        {
            start[axis] = end[axis] - diff[axis] + 1;
        }
        else
        {
            end[axis] = start[axis] - diff[axis] - 1;
        }
        save_load_area(start, end, false);
    }
}

void LocalMap::save_load_area(const Vector3i& start, const Vector3i& end, bool save)
{
    // explanation: We only want to touch each Chunk once,
    // instead of every time that get/set_value is called.
    // => iterate over all affected Chunks and handle all values within each Chunk

    constexpr int CHUNK_SIZE = GlobalMap::CHUNK_SIZE;

    Vector3i chunk_start = floor_shift(start, GlobalMap::CHUNK_SHIFT);
    Vector3i chunk_end = floor_shift(end, GlobalMap::CHUNK_SHIFT);

    // The start and end point within chunk_start and chunk_end
    Vector3i start_delta = start - chunk_start * CHUNK_SIZE;
    Vector3i end_delta = end - chunk_end * CHUNK_SIZE;
    for (int chunk_x = chunk_start.x(); chunk_x <= chunk_end.x(); ++chunk_x)
    {
        for (int chunk_y = chunk_start.y(); chunk_y <= chunk_end.y(); ++chunk_y)
        {
            for (int chunk_z = chunk_start.z(); chunk_z <= chunk_end.z(); ++chunk_z)
            {
                auto& chunk = map_->activate_chunk(Vector3i(chunk_x, chunk_y, chunk_z));

                // if we are in the starting chunk, start there. Otherwise take the entire Chunk
                int dx_start = chunk_x == chunk_start.x() ? start_delta.x() : 0;
                int dy_start = chunk_y == chunk_start.y() ? start_delta.y() : 0;
                int dz_start = chunk_z == chunk_start.z() ? start_delta.z() : 0;
                // if we are in the ending chunk, end there. Otherwise take the entire Chunk
                int dx_end = chunk_x == chunk_end.x() ? end_delta.x() : CHUNK_SIZE - 1;
                int dy_end = chunk_y == chunk_end.y() ? end_delta.y() : CHUNK_SIZE - 1;
                int dz_end = chunk_z == chunk_end.z() ? end_delta.z() : CHUNK_SIZE - 1;

                Vector3i global_pos;
                int index_x, index_y, index;
                for (int dx = dx_start; dx <= dx_end; ++dx)
                {
                    index_x = dx * CHUNK_SIZE * CHUNK_SIZE;
                    global_pos.x() = chunk_x * CHUNK_SIZE + dx;

                    for (int dy = dy_start; dy <= dy_end; ++dy)
                    {
                        index_y = index_x + dy * CHUNK_SIZE;
                        global_pos.y() = chunk_y * CHUNK_SIZE + dy;

                        for (int dz = dz_start; dz <= dz_end; ++dz)
                        {
                            index = index_y + dz;
                            global_pos.z() = chunk_z * CHUNK_SIZE + dz;

                            if (save)
                            {
                                chunk[index] = value_unchecked(global_pos).raw();
                            }
                            else
                            {
                                value_unchecked(global_pos).raw(chunk[index]);
                            }
                        }
                    }
                }
            }
        }
    }
}

buffer::InputOutputBuffer<TSDFValue>& LocalMap::getBuffer()
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
    Vector3i start = pos_ - size_ / 2;
    Vector3i end = pos_ + size_ / 2;
    save_load_area(start, end, true);

    map_->write_back();
}

} // namespace fastsense::map
