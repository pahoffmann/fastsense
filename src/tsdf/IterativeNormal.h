#pragma once

/**
 * @file IterativeNormal.h
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <vector>

#include <tsdf/FittingNormalBase.h>

namespace fastsense::tsdf
{

/**
 * @brief Class for representing a normel determination approach where a plane is fitted in a neighborhood
 *        of points around the query point by minimizing the least square error
 *
 * @tparam POINT_T Data type of the points
 * @tparam SET_T Data type of the point set
 */
template<typename POINT_T, typename SET_T>
class IterativeNormal : public FittingNormalBase<POINT_T, SET_T, std::vector<POINT_T>>
{
public:
    /**
     * @brief Construct a new Iterative Normal object
     *
     * @param neighbors Helper object for determining neighbors of a given point
     */
    IterativeNormal(std::shared_ptr<NeighborsBase<POINT_T, SET_T, std::vector<POINT_T>>>& neighbors) : FittingNormalBase<POINT_T, SET_T, std::vector<POINT_T>>(neighbors) {}

    /**
     * @brief Functor for calculation the normal of given point in a given point set
     *
     * @param query_ring Scan ring at which the query point can be found
     * @param query_index Index at which the query point can be found on the given scan ring
     * @param points Set of points to be used for the determination of the normal
     *
     * @return The normal of the given point based on a given point set
     *
     * @throws -1 If the calculated determinants are invalid
     */
    virtual POINT_T operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const override;
};

} // namespace fastsense::tsdf

#include <tsdf/IterativeNormal.tcc>
