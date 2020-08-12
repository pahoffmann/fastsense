/**
 * @author Marcel Flottmann
 * @date   2020-08-11
 */

#pragma once

#include <cstdint>
#include <vector>
#include <memory>

struct point
{
    float x;
    float y;
    float z;
    float intensity;
    uint32_t ring;
};

class point_cloud
{
public:
    using ptr = std::shared_ptr<point_cloud>;
    std::vector<point> points;
};