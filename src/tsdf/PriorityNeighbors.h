#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

#include <tsdf/NeighborsBase.h>

#include <vector>

namespace fastsense::tsdf
{

/**
 * @brief Class for representing a neighbor determination approach where neighbor points
 *        are choosen based on a priorety queue. In this abbstract data type,
 *        every considered neighbor is inserted and sorted by the distance to the query point.
 *        In every iteration, the point with the minimal distance from the query point is choosen and deleted
 *        and every direct neighbor is inserted in the queue until the neighborhood is complete
 *
 * @tparam POINT_T Data type of the points
 * @tparam SET_T Data type of the point set
 */
template<typename POINT_T, typename SET_T>
class PriorityNeighbors : public NeighborsBase<POINT_T, SET_T, std::vector<POINT_T>>
{
public:
    /**
     * @brief Construct a new Priority Neighbors object
     *
     * @param number_of_neighbors Number of neighbors that should be determined
     */
    PriorityNeighbors(unsigned int number_of_neighbors) : number_of_neighbors_(number_of_neighbors) {}

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
    /// Number of neighbors that should be determined
    unsigned short number_of_neighbors_;

    /**
     * @brief Struct for ordering the point indices in a set and a priority queue
     */
    struct PairComparator
    {
        /**
         * @brief Functor for priority queue object
         */
        bool operator()(const std::pair<double, std::pair<short, unsigned int>>& lhs, const std::pair<double, std::pair<short, unsigned int>>& rhs)
        {
            return lhs.first > rhs.first;
        }

        /**
         * @brief Functor for set object
         */
        bool operator()(const std::pair<short, unsigned int>& lhs, const std::pair<short, unsigned int>& rhs)
        {
            if (lhs.first == rhs.first)
            {
                return lhs.second < rhs.second;
            }

            return lhs.first < rhs.first;
        }
    };
};

} // namespace fastsense::tsdf

#include <tsdf/PriorityNeighbors.tcc>
