#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

#include <tsdf/FittingNormalBase.h>

namespace fastsense::tsdf
{

/**
 * @brief Class for representing a normel determination approach where a plane is fitted in a neighborhood 
 *        of points around the query point by using RANSAC
 * 
 * @tparam POINT_T Data type of the points
 * @tparam SET_T Data type of the point set
 */
template<typename POINT_T, typename SET_T>
class RANSACNormal : public FittingNormalBase<POINT_T, SET_T, std::vector<POINT_T>>
{
public:
    /**
     * @brief Construct a new RANSACNormal object
     * 
     * @param neighbors Helper object for determining neighbors of a given point
     * @param max_iterations Maximum iterations where a better plane should be found
     * @param max_nonimpiter Maximum iterations without an improvement for the estimated plane
     */
    RANSACNormal(std::shared_ptr<NeighborsBase<POINT_T, SET_T, std::vector<POINT_T>>>& neighbors, unsigned int max_iterations = 20,
                 unsigned int max_nonimpiter = 10) : FittingNormalBase<POINT_T, SET_T, std::vector<POINT_T>>(neighbors), max_iterations_(max_iterations), max_nonimpiter_(max_nonimpiter) {}

    /**
     * @brief Functor for calculation the normal of given point in a given point set
     *
     * @param query_ring Scan ring at which the query point can be found
     * @param query_index Index at which the query point can be found on the given scan ring
     * @param points Set of points to be used for the determination of the normal
     *
     * @return The normal of the given point based on a given point set
     * 
     * @throw -1 If no neighbors could be found or a deadlock occured while choosing a new plane randomly
     */
    virtual POINT_T operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const override;

private:
    unsigned int max_iterations_;
    unsigned int max_nonimpiter_;
};

} // namespace fastsense::tsdf

#include <tsdf/RANSACNormal.tcc>
