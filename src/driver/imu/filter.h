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
class SlidingWindowFilter : public Filter<T>
{
public:
    explicit SlidingWindowFilter(size_t window_size)
    : Filter<T>{}
    , window_size_{static_cast<double>(window_size)}
    , buffer_{}
    , count_{}
    , mean_{}
    {}

    /**
     * Performs update of average with new data across dynamic window:
     * according to: https://de.wikipedia.org/wiki/Gleitender_Mittelwert#Gleitender_Durchschnitt_mit_dynamischem_Fenster
     *
     * @param new_value type T
     * @return updated mean
     */
    const T &update(const T &new_value)
    {
        if (window_size_ < 2)
        {
            return new_value;
        }

        buffer_.push(new_value);
        if (buffer_.size() <= window_size_)
        {
            mean_ += (new_value / window_size_);
            return new_value;
        }

        mean_ += (buffer_.back() - buffer_.front())/window_size_;
        buffer_.pop();
        return mean_;
    }

    const std::queue<T>& get_buffer() const
    {
        return buffer_;
    }

    const T& get_mean() const
    {
        return mean_;
    }

private:
    double window_size_;
    std::queue<T> buffer_;
    double count_;
    T mean_;
};

} // namespace fastsense::driver