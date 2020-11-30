#pragma once

/**
 * @file stamped.h
 * @author Julian Gaal
 */

#include "../util/time.h"

namespace fastsense::msg {

template<typename DATA_T>
struct Stamped
{
    Stamped()
    : data_{}
    , timestamp_{util::HighResTime::now()}
    {
    }

    explicit Stamped(DATA_T data, util::HighResTimePoint timepoint = util::HighResTime::now())
    : data_(std::move(data))
    , timestamp_(timepoint)
    {
    }

    virtual ~Stamped() = default;

    DATA_T data_;
    util::HighResTimePoint timestamp_;
};

}