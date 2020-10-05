#pragma once

/**
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

namespace fastsense::tsdf
{

/**
 * @brief Base class for determining the neighbo points of a give npoint in a given point set
 *
 * @tparam POINT_T Data type of the points
 * @tparam SET_T Data type of the point set
 * @tparam NEIGHBORS_T Data type of the Neighborhood
 */
template<typename POINT_T, typename SET_T, typename NEIGHBORS_T>
class NeighborsBase
{
public:
    /**
     * @brief Functor for determining the neighbors of given point in a given point set
     *
     * @param query_ring Scan ring at which the query point can be found
     * @param query_index Index at which the query point can be found on the given scan ring
     * @param points Set of points in which the neighbors should be determined
     *
     * @return The Neighbors of the given point based on a given point set
     */
    virtual NEIGHBORS_T operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const = 0;

    virtual ~NeighborsBase() = default;
};

} // namespace fastsense::tsdf