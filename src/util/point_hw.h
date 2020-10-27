#pragma once

/**
 * @file point_hw.h
 * @author Marcel Flottmann
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include "hls_functions.h"

#include <util/constants.h>

struct PointHW
{
    int x;
    int y;
    int z;
    int dummy; //128 bit padding

    PointHW() : x(0), y(0), z(0)
    {}

    PointHW(int x, int y, int z)
        : x(x), y(y), z(z)
    {}

    PointHW(const PointHW& rhs)
    {
        *this = rhs;
    }

    PointHW operator+(const PointHW& rhs) const
    {
        PointHW p;
        p.x = x + rhs.x;
        p.y = y + rhs.y;
        p.z = z + rhs.z;
        return p;
    }

    PointHW operator-(const PointHW& rhs) const
    {
        PointHW p;
        p.x = x - rhs.x;
        p.y = y - rhs.y;
        p.z = z - rhs.z;
        return p;
    }

    PointHW operator*(int rhs) const
    {
        PointHW p;
        p.x = x * rhs;
        p.y = y * rhs;
        p.z = z * rhs;
        return p;
    }

    PointHW operator/(int rhs) const
    {
        PointHW p;
        p.x = x / rhs;
        p.y = y / rhs;
        p.z = z / rhs;
        return p;
    }

    PointHW& operator=(const PointHW& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }

    PointHW& operator=(int rhs)
    {
        x = rhs;
        y = rhs;
        z = rhs;
        return *this;
    }

    bool operator==(const PointHW& p) const
    {
        return x == p.x && y == p.y && z == p.z;
    }

    int norm2() const
    {
        return x * x + y * y + z * z;
    }

    int norm() const
    {
        return hls_sqrt_approx(norm2());
    }

    PointHW abs() const
    {
        PointHW p;
        p.x = hls_abs(x);
        p.y = hls_abs(y);
        p.z = hls_abs(z);
        return p;
    }

    PointHW sign() const
    {
        PointHW p;
        p.x = x < 0 ? -1 : 1;
        p.y = y < 0 ? -1 : 1;
        p.z = z < 0 ? -1 : 1;
        return p;
    }

    PointHW to_map() const
    {
        PointHW p;
        p.x = x / MAP_RESOLUTION;
        p.y = y / MAP_RESOLUTION;
        p.z = z / MAP_RESOLUTION;
        return p;
    }

    PointHW to_mm() const
    {
        PointHW p;
        p.x = (x * MAP_RESOLUTION) + MAP_RESOLUTION / 2;
        p.y = (y * MAP_RESOLUTION) + MAP_RESOLUTION / 2;
        p.z = (z * MAP_RESOLUTION) + MAP_RESOLUTION / 2;
        return p;
    }
};
