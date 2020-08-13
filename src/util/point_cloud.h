/**
 * @author Marcel Flottmann
 * @date   2020-08-11
 */

#pragma once

#include <cstdint>
#include <vector>
#include <memory>

struct Point
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

class PointCloud
{
public:
    using ptr = std::shared_ptr<PointCloud>;
    std::vector<Point> points;
    uint16_t rings;
};
