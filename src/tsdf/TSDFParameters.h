#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

#include <iostream>

namespace fastsense::tsdf
{

/**
 * @brief Class that encapsulate the handling of the different parameters for the TSDF calculation and map representation
 *
 * @tparam SCALAR_T Data type of the scalar variables
 * @tparam VEC_T Data type of the vector variables
 */
template<typename SCALAR_T, typename VEC_T>
class TSDFParameters
{
public:
    /**
     * @brief Construct a new TSDFParameters object
     *
     * @param map_resolution Resolution of the TSDF map in meters per cell
     * @param map_size Size of the map for each dimension in meters
     * @param max_distance Truncation for the TSDF values in the map (in meters)
     */
    TSDFParameters(SCALAR_T map_resolution = 0.03, const VEC_T& map_size = VEC_T(20.0, 20.0, 5.0), SCALAR_T max_distance = (0.5 / 0.03)) :
        max_distance_(max_distance),
        map_resolution_(map_resolution),
        map_size_(map_size)
    {
        updateParams();
    }

    /// Set the truncation for the TSDF values in the map (in meters)
    void setMaxDistance(SCALAR_T max_distance)
    {
        max_distance_ = max_distance;
        updateTau();
    }

    /// Get the truncation for the TSDF values in the map (in meters)
    SCALAR_T getMaxDistance()
    {
        return max_distance_;
    }

    /// Get the truncation for the TSDF values in the map (in grid units)
    SCALAR_T getTau() const
    {
        return tau_;
    }

    /// Set the resolution of the TSDF map in meters per cell
    void setMapResolution(SCALAR_T map_resolution)
    {
        map_resolution_ = map_resolution;
        updateParams();
    }

    /// Get the resolution of the TSDF map in meters per cell
    SCALAR_T getMapResolution() const
    {
        return map_resolution_;
    }

    /// Set the size of the map for each dimension in meters
    void setMapSize(const VEC_T& map_size)
    {
        map_size_ = map_size;
        updateParams();
    }

    /// Get the size of the map for each dimension in meters
    const VEC_T& getMapSize() const
    {
        return map_size_;
    }

    /// Get the position of the scanner in the map (in grid units)
    const VEC_T& getScannerPos() const
    {
        return scanner_pos_;
    }

    /// Get the map size of the x dimension in grid units
    size_t getMapGridSizeX() const
    {
        return map_grid_size_x_;
    }

    /// Get the map size of the y dimension in grid units
    size_t getMapGridSizeY() const
    {
        return map_grid_size_y_;
    }

    /// Get the map size of the z dimension in grid units
    size_t getMapGridSizeZ() const
    {
        return map_grid_size_z_;
    }

    /// Get the total size of the map in grid units
    size_t getTotalSize() const
    {
        return total_size_;
    }

    template<typename S, typename V>
    friend std::ostream& operator<<(std::ostream& os, const TSDFParameters<S, V>& params);

private:
    /// Truncation for the TSDF values in the map (in meters)
    SCALAR_T max_distance_;
    /// Truncation for the TSDF values in the map (in grid units)
    SCALAR_T tau_;
    /// Resolution of the TSDF map in meters per cell
    SCALAR_T map_resolution_;
    /// Size of the map for each dimension in meters
    VEC_T map_size_;
    /// Position of the scanner in the map (in grid units)
    VEC_T scanner_pos_;
    /// Map size of the x dimension in grid units
    size_t map_grid_size_x_;
    /// Map size of the y dimension in grid units
    size_t map_grid_size_y_;
    /// Map size of the z dimension in grid units
    size_t map_grid_size_z_;
    /// Total size of the map in grid units
    size_t total_size_;

    /**
     * @brief Update tau based on the max distance
     */
    void updateTau()
    {
        tau_ = max_distance_ / map_resolution_;
    }

    /**
     * @brief Making all map parameters consitence
     */
    void updateMap();

    /**
     * @brief Making all stored parameters consistence
     */
    void updateParams()
    {
        updateMap();
        updateTau();
    }
};

/**
 * @brief Sending different paramter information over an output stream
 *
 * @param os Output stream to which the parameter information is to be sent
 * @param params Contains parameters, which should be sent to the stream
 * @return std::ostream& Providing concatination of the stream operator
 *
 * @tparam SCALAR_T Data type of the scalar variables
 * @tparam VEC_T Data type of the vector variables
 */
template<typename SCALAR_T, typename VEC_T>
std::ostream& operator<<(std::ostream& os, const TSDFParameters<SCALAR_T, VEC_T>& params);

} // namespace fastsense::tsdf

#include <tsdf/TSDFParameters.tcc>
