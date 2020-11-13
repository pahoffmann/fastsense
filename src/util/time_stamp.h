#pragma once

/**
 * @file time_stamp.h
 * @author Julian Gaal
 */

#include <chrono>
#include <tuple>

namespace fastsense::util
{

template <typename TIME_UNIT>
struct TimeStamp
{
    TimeStamp() : time(std::chrono::steady_clock::now()) {}

    int operator-(const TimeStamp& t)
    {
        return std::chrono::duration_cast<TIME_UNIT>(time - t.time).count();
    }

    void operator+=(const std::chrono::duration<int> diff)
    {
        time += diff;
    }

    void reset()
    {
        time = std::chrono::steady_clock::now();
    }

    std::chrono::steady_clock::time_point time;
};



template<typename TIME_UNIT>
class TimeDiff
{
public:
    TimeDiff(const TimeStamp<TIME_UNIT>& new_time, const TimeStamp<TIME_UNIT>& old_time)
    {

    }

    std::chrono::duration<int> getDiff() const 
    {
        
    }

} // namespace fastsense::util