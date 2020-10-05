/**
 * @author Julian Gaal
 */

#include <util/time_stamp.h>

using namespace fastsense::util;

TimeStamp::TimeStamp() : time(std::chrono::steady_clock::now()) {}

int TimeStamp::operator-(const TimeStamp& t)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(time - t.time).count();
}

void TimeStamp::reset()
{
    time = std::chrono::steady_clock::now();
}


