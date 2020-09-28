#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

#include <tsdf/NormalBase.h>

namespace fastsense::tsdf
{

/**
 * @brief Class for representing a normal determination approach where an average upper, lower, left and right point 
 *        are calculated using the scan order. Based on these points, 
 *        the cross product is then applied to determine the normal of the query point 
 * 
 * @tparam POINT_T Data type of the points
 * @tparam SET_T Data type of the point set
 */
template<typename POINT_T, typename SET_T>
class AverageNormal : public NormalBase<POINT_T, SET_T>
{
public:
    /**
     * @brief Construct a new Average Normal object
     * 
     * @param neighbor_range Number of iteration in which a new neighbor point should be considered  
     *                       and possibly added to the average points
     * @param max_dist Upper bound for the distance of an average point to thequery point
     *                 If a new potential neighbor point results in the limit being exceeded,
     *                 the point will be discarded
     */
    AverageNormal(unsigned int neighbor_range, double max_dist) : neighbor_range_(neighbor_range), max_dist_(max_dist) {}
    
    /**
     * @brief Functor for calculation the normal of given point in a given point set
     *
     * @param query_ring Scan ring at which the query point can be found
     * @param query_index Index at which the query point can be found on the given scan ring
     * @param points Set of points to be used for the determination of the normal
     *
     * @return The normal of the given point based on a given point set
     * 
     * @throws -1 If no upper or lower points could be found or if the number of left and right points is one each
     */
    virtual POINT_T operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const override;

private:
    /// Number of iteration in which a new neighbor point should be considered and possibly added to the average points
    unsigned int neighbor_range_;
    
    /// Upper bound for the distance of an average point to thequery point
    /// If a new potential neighbor point results in the limit being exceeded, the point will be discarded
    double max_dist_;
};

} // namespace fastsense::tsdf

#include <tsdf/AverageNormal.tcc>
