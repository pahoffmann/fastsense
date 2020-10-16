#pragma once

/**
 * @file NormalBase.h
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

namespace fastsense::tsdf
{

/**
 * @brief Base class for determining the normals for a give point in a set of points
 *
 * @tparam POINT_T Datatype of the points
 * @tparam SET_T  Datatype of the point set
 */
template<typename POINT_T, typename SET_T>
class NormalBase
{
public:
    /**
     * @brief Functor for calculation the normal of given point in a given point set
     *
     * @param query_ring Scan ring at which the query point can be found
     * @param query_index Index at which the query point can be found on the given scan ring
     * @param points Set of points to be used for the determination of the normal
     *
     * @return The normal of the given point based on a given point set
     */
    virtual POINT_T operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const = 0;

    virtual ~NormalBase() = default;
};

} // namespace fastsense::tsdf