#pragma once

/**
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include "NormalBase.h"

namespace fastsense::tsdf
{

/**
 * @brief Class for representing the determination of the direction vector from the scanner position to the scan point.
 *
 * @tparam POINT_T Data type of the points
 * @tparam SET_T Data type of the point set
 */
template<typename POINT_T, typename SET_T>
class ProjectionNormal : public NormalBase<POINT_T, SET_T>
{
public:
    /**
     * @brief Construct a new Projection Normal object
     *
     * @param scanner_pos Origin of the scanner
     */
    ProjectionNormal(const POINT_T& scanner_pos) : scanner_pos_(scanner_pos) {}

    /**
     * @brief Functor for calculation the normal of given point in a given point set
     *
     * @param query_ring Scan ring at which the query point can be found
     * @param query_index Index at which the query point can be found on the given scan ring
     * @param points Set of points to be used for the determination of the normal
     *
     * @return The normal of the given point based on a given point set
     *
     * @throw -1 If the calculated determinants are invalid
     */
    virtual POINT_T operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const override
    {
        return (points[query_ring][query_index] - scanner_pos_).normalized();
    }

private:
    /// Origin of the scanner
    POINT_T scanner_pos_;
};

} // namespace fastsense::tsdf
