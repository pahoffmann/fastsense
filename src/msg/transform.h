#pragma once

/**
 * @file transform.h
 * @author Marcel Flottmann
 */

#include <util/point.h>
#include <msg/stamped.h>
#include <util/concurrent_ring_buffer.h>

namespace fastsense::msg
{

struct Transform
{
    Transform() 
    : rotation{0}
    , translation{Vector3f::Zero()}
    {}

    Transform(Quaternionf q, Vector3f t)
    : rotation{q}
    , translation{t}
    {}

    ~Transform() = default;

    Quaternionf rotation;
    Vector3f translation;
};

using TransformStamped = msg::Stamped<Transform>;
using TransformStampedBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::TransformStamped>;

} // namespace fastsense::msg