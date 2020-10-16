#pragma once

/**
 * @file ScanOrderNeighbors.h
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <vector>

#include "NeighborsBase.h"

namespace fastsense::tsdf
{

/**
 * @brief Class for representing a neighbor determination approach where neighbor points
 *        are only choosen from the scan order in the given point set
 *
 * @tparam POINT_T Data type of the points
 * @tparam SET_T Data type of the point set
 */
template<typename POINT_T, typename SET_T>
class ScanOrderNeighbors : public NeighborsBase<POINT_T, SET_T, std::vector<POINT_T>>
{
public:
    /**
     * @brief Construct a new Scan Order Neighbors object
     *
     * @param neighbor_rings How many surrounding scan rings should be considered in every direction?
     * @param neighbor_range How many surrounding points should be considered on a ring in every direction?
     */
    ScanOrderNeighbors(unsigned short neighbor_rings, int neighbor_range) : neighbor_rings_(neighbor_rings), neighbor_range_(neighbor_range) {}

    /**
     * @brief Functor for determining the neighbors of given point in a given point set
     *
     * @param query_ring Scan ring at which the query point can be found
     * @param query_index Index at which the query point can be found on the given scan ring
     * @param points Set of points in which the neighbors should be determined
     *
     * @return The Neighbors of the given point based on a given point set
     */
    virtual std::vector<POINT_T> operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const override;

private:
    /// How many surrounding scan rings should be considered in every direction?
    unsigned short neighbor_rings_;

    /// How many surrounding points should be considered on a ring in every direction?
    int neighbor_range_;
};

} // namespace fastsense::tsdf

#include "ScanOrderNeighbors.tcc"
