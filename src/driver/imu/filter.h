#pragma once

#include <util/concurrent_ring_buffer.h>
#include <queue>

namespace fastsense::driver
{

template<typename T>
class Filter
{
protected:
    Filter() = default;
    virtual ~Filter() = default;
    virtual const T &update(const T &new_value) = 0;
};

template<typename T>
class MovingAverageFilter : public Filter<T>
{
public:
    MovingAverageFilter() : Filter<T>{}, count_{}, mean_{}
    {}

    /**
     * Performs update of average with new data:
     * Implements new_mean = old_mean + (new_data - old_mean) / n_msg
     *
     * @param new_value type T
     * @return updated mean
     */
    const T &update(const T &new_value)
    {
        ++count_;
        const auto differential = (new_value - mean_) / count_;
        mean_ += differential;
        return mean_;
    }

private:
    double count_;
    T mean_;
};

template<typename T>
class RecursiveAverageFilter : public Filter<T>
{
public:
    RecursiveAverageFilter()
    : Filter<T>{}
    , window_size_{5}
    , p{static_cast<size_t>((window_size_ - 1)/2)}
    , q{p+1}
    , buffer_{window_size_ + 1}
    , count_{0}
    , mean_{}
    {
        assert(p == 2)
        assert(q == p+1);
    }

    /**
     * Performs update of average with new data:
     *
     *
     * @param new_value type T
     * @return updated mean
     */
    const T &update(const T &new_value)
    {
        if (window_size_ == 0 || window_size_ == 2)
        {
            return new_value;
        }

        // Sums up each measurement (before first mean is calculated, initialization)
        // Return empty imu message during that time
        if (count_ <= window_size_-1)
        {
            mean_ += new_value;
            ++count_;
            return T{};
        }

        // Calculate first average, once window_size_ messages have been received
        if (count_ == window_size_-1)
        {
            mean_ /= count_;
            return mean_;
        }

        buffer_.push(new_value);

        // Buffer needs to be window_size_ + 1 in size
        // before this algorithm works correctly
        // example: window of size 7
        //
        // x: measurements
        // 47 48 49 50 51 52 53 54
        //
        // mean[51] = last_mean + (x[54] - x[47])/window_size
        //                aka buffer.back() - buffer.front()/pop()
        //
        // So, if buffer is still too small, return current mean
        if (buffer_.size() != window_size_+1)
        {
            return mean_;
        }

        mean_ = mean_ + (buffer_.back() - buffer_.pop())/window_size_;
        return mean_;
    }

private:
    double window_size_;
    size_t p;
    size_t q;
    std::queue<T> buffer_;
    double count_;
    T mean_;
};

} // namespace fastsense::driver