#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

#include <math.h>

namespace fastsense::tsdf
{

/**
 * @brief Base class for weighting tsdf values based on surface distance only
 * 
 * @tparam T Data type of the use variables and the return value
 */
template<typename T>
class WeightingBase
{
public:
    /**
     * @brief Functor that calculates a weight based on the given distance to the surface 
     * 
     * @param d Distance to the surface
     * @return T Weight for the correspinding tsdf grid
     */
    virtual T operator()(const T d) const = 0;
};

/**
 * @brief Class that represents an exponential decrease approach. 
 *        After a specific distance to the surface, 
 *        the weight decreases exponential until the truncation value is reached.
 *        After the truncation value is exceeded, the weight is set to zero 
 * 
 * @tparam T Data type of the use variables and the return value
 */
template<typename T>
class ExponentialWeight : public WeightingBase<T>
{
public:
    /**
     * @brief Construct a new Exponential Weight object
     * 
     * @param tau Truncation value for the tsdf values
     * @param epsilon If the distance is lower than this value, the weight is set to one. 
     *        Otherwise it will be decreased based on the approach
     * @param sigma Standard deviation for the exponential decrease (like gaussian noise)
     */
    ExponentialWeight(T tau, T epsilon, T sigma) : tau_(tau), epsilon_(epsilon), sigma_(sigma) {}

    /**
     * @brief Functor that calculates a weight based on the given distance to the surface 
     * 
     * @param d Distance to the surface
     * @return T Weight for the correspinding tsdf grid
     */
    virtual T operator()(const T d) const override
    {
        if (d < epsilon_)
        {
            return 1.0;
        }

        if (d >= epsilon_ && d < tau_)
        {
            double diff = d - epsilon_;
            return exp(-sigma_ * diff * diff);
        }

        return 0.0;
    }

private:
    /// Standard deviation for the exponential decrease (like gaussian noise)
    T sigma_;
    /// Truncation value for the tsdf values
    T tau_;
    /// If the distance is lower than this value, the weight is set to one. 
    /// Otherwise it will be decreased based on the approach
    T epsilon_;
};

/**
 * @brief Class that represents an narrow exponential decrease approach. 
 *        Before and after a specific distance to the surface, 
 *        the weight decreases exponential until the truncation value is reached.
 *        Before and after the truncation value is exceeded, the weight is set to zero.
 *        The difference to the (normal) exponential approach is, 
 *        that the distance value is cosidered as absolute value 
 * 
 * @tparam T Data type of the use variables and the return value
 */
template<typename T>
class NarrowExponentialWeight : public ExponentialWeight<T>
{
public:
    /**
     * @brief Construct a new Narrow Exponential Weight object
     * 
     * @param tau Truncation value for the tsdf values
     * @param epsilon If the distance is lower than this value, the weight is set to one. 
     *        Otherwise it will be decreased based on the approach
     * @param sigma Standard deviation for the exponential decrease (like gaussian noise)
     */
    NarrowExponentialWeight(T tau, T epsilon, T sigma) : ExponentialWeight<T>(tau, epsilon, sigma) {}

    /**
     * @brief Functor that calculates a weight based on the given distance to the surface 
     * 
     * @param d Distance to the surface
     * @return T Weight for the correspinding tsdf grid
     */
    virtual T operator()(const T d) const override
    {
        ExponentialWeight<T>::operator()(fabs(d));
    }
};

/**
 * @brief Class that represents an linear decrease approach. 
 *        After a specific distance to the surface, 
 *        the weight decreases linear until the truncation value is reached.
 *        After the truncation value is exceeded, the weight is set to zero 
 * 
 * @tparam T Data type of the use variables and the return value
 */
template<typename T>
class LinearWeight : public WeightingBase<T>
{
public:
    /**
     * @brief Construct a new Linear Weight object
     * 
     * @param tau Truncation value for the tsdf values
     * @param epsilon If the distance is lower than this value, the weight is set to one. 
     *        Otherwise it will be decreased based on the approach
     */
    LinearWeight(T tau, T epsilon) : tau_(tau), epsilon_(epsilon) {}

    /**
     * @brief Functor that calculates a weight based on the given distance to the surface 
     * 
     * @param d Distance to the surface
     * @return T Weight for the correspinding tsdf grid
     */
    virtual T operator()(const T d) const override
    {
        if (d < epsilon_)
        {
            return 1.0;
        }

        if (d >= epsilon_ && d <= tau_)
        {
            return (tau_ - d) / (tau_ - epsilon_);
        }

        return 0.0;
    }

private:
    /// Truncation value for the tsdf values
    T tau_;
    /// If the distance is lower than this value, the weight is set to one. 
    /// Otherwise it will be decreased based on the approach
    T epsilon_;
};

/**
 * @brief Class that represents an narrow linear decrease approach. 
 *        Before and after a specific distance to the surface, 
 *        the weight decreases linear until the truncation value is reached.
 *        Before and after the truncation value is exceeded, the weight is set to zero.
 *        The difference to the (normal) exponential approach is, 
 *        that the distance value is cosidered as absolute value 
 * 
 * @tparam T Data type of the use variables and the return value
 */
template<typename T>
class NarrowLinearWeight : public LinearWeight<T>
{
public:
    /**
     * @brief Construct a new Narrow Linear Weight object
     * 
     * @param tau Truncation value for the tsdf values
     * @param epsilon If the distance is lower than this value, the weight is set to one. 
     *        Otherwise it will be decreased based on the approach
     */
    NarrowLinearWeight(T tau, T epsilon) : LinearWeight<T>(tau, epsilon) {}

    /**
     * @brief Functor that calculates a weight based on the given distance to the surface 
     * 
     * @param d Distance to the surface
     * @return T Weight for the correspinding tsdf grid
     */
    virtual T operator()(const T d) const override
    {
        LinearWeight<T>::operator()(fabs(d));
    }
};

} // namespace fastsense::tsdf
