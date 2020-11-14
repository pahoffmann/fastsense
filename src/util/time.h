#pragma once

#include <chrono>

namespace fastsense::util
{

using HighResTime = std::chrono::high_resolution_clock;
using HighResTimePoint = HighResTime::time_point;

namespace time
{
using secs_float = std::chrono::duration<float>;
}

} // namespace time