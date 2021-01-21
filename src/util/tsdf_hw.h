#pragma once

/**
 * @file tsdf.h
 * @author Marcel Flottmann
 * @date 2021-01-14
 */

#include <cinttypes>
#include <util/constants.h>

using ValueType = int16_t;
using WeightType = uint16_t;
struct TSDFValueHW
{
    // *INDENT-OFF*
    ValueType value : VALUE_BITS;
    WeightType weight : WEIGHT_BITS;
    // *INDENT-ON*
};
