#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

#include "prototyping/tsdf_values/NormalBase.h"
#include "prototyping/tsdf_values/NeighborsBase.h"

#include <memory>

/**
 * @brief Base class for determining the normals by fitting a plane in a neighborhood of points
 * 
 * @tparam POINT_T Data type of the points
 * @tparam SET_T Data type of the point set
 * @tparam NEIGHBOR_T Data type of the Neighborhood
 */
template<typename POINT_T, typename SET_T, typename NEIGHBOR_T>
class FittingNormalBase : public NormalBase<POINT_T, SET_T>
{
protected:
    /**
     * @brief Construct a new Fitting Normal Base object
     * 
     * @param neighbors Helper object for determining neighbors of a given point
     */
    FittingNormalBase(std::shared_ptr<NeighborsBase<POINT_T, SET_T, NEIGHBOR_T>>& neighbors) : neighbors_(neighbors) {}

    /// Represents how neighbors of a point should be determined
    std::shared_ptr<NeighborsBase<POINT_T, SET_T, NEIGHBOR_T>> neighbors_;
};
