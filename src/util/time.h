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
	static void reset()
	{
		[[maybe_unused]]
		auto& time = getInst();
		time.set_time(HighResTime::now());
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
	
	~RelativeTime() = default;

private:
	RelativeTime() : start_time{HighResTime::now()} {}
	
	void set_time(const HighResTimePoint& t)
	{
		start_time = t;
	}

	static RelativeTime& getInst()
	{
		static RelativeTime time;
		return time;
	}
	
	HighResTimePoint start_time;
};

} // namespace fastsense::util