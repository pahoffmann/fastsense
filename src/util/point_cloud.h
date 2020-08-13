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
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

class point_cloud
{
public:
    using ptr = std::shared_ptr<point_cloud>;
    std::vector<point> points;
    uint16_t rings;
};