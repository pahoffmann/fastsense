#pragma once

#include <chrono>

namespace fastsense::util
{

using HighResTime = std::chrono::high_resolution_clock;
using HighResTimePoint = HighResTime::time_point;

using secs_double = std::chrono::duration<double>;

class RelativeTime
{
public:
	static void init()
	{
		[[maybe_unused]]
		auto& time = getInst();
	}
	
	static uint64_t now()
	{
		const auto& time = getInst();
		return std::chrono::duration_cast<std::chrono::milliseconds>(HighResTime::now() - time.start_time).count();
	}
	
	RelativeTime& operator=(RelativeTime&&) = delete;
	RelativeTime& operator=(const RelativeTime&) = delete;
	
	RelativeTime(RelativeTime&&) = delete;
	RelativeTime(const RelativeTime&) = delete;
	
private:
	RelativeTime() : start_time{HighResTime::now()} {}
	
	~RelativeTime() = default;
	
	static RelativeTime& getInst()
	{
		static RelativeTime time;
		return time;
	}
	
	const HighResTimePoint start_time;
};

} // namespace fastsense::util