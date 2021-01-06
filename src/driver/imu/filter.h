#pragma once

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

} // namespace fastsense::driver