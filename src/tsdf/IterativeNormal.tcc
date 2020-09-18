#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

namespace fastsense::tsdf
{

template<typename POINT_T, typename SET_T>
POINT_T IterativeNormal<POINT_T, SET_T>::operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const
{
    auto neighbors = (*(this->neighbors_))(query_ring, query_index, points);
    const auto& queryPoint = points[query_ring][query_index];

    size_t numPoints = neighbors.size();

    //x
    float xx = 0.0;
    float xy = 0.0;
    float xz = 0.0;

    //y
    float yy = 0.0;
    float yz = 0.0;

    //z
    float zz = 0.0;

    for (int j = 0; j < numPoints; j++)
    {
        auto r = neighbors[j] - queryPoint;

        xx += r.x() * r.x();
        xy += r.x() * r.y();
        xz += r.x() * r.z();
        yy += r.y() * r.y();
        yz += r.y() * r.z();
        zz += r.z() * r.z();
    }

    //determinante
    float det_x = yy * zz - yz * yz;
    float det_y = xx * zz - xz * xz;
    float det_z = xx * yy - xy * xy;

    float dir_x;
    float dir_y;
    float dir_z;
    // det X biggest
    if (det_x >= det_y && det_x >= det_z)
    {

        if (det_x <= 0.0)
        {
            throw (-1);
        }

        dir_x = 1.0;
        dir_y = (xz * yz - xy * zz) / det_x;
        dir_z = (xy * yz - xz * yy) / det_x;
    } //det Y biggest
    else if ( det_y >= det_x && det_y >= det_z)
    {

        if (det_y <= 0.0)
        {
            throw (-1);
        }

        dir_x = (yz * xz - xy * zz) / det_y;
        dir_y = 1.0;
        dir_z = (xy * xz - yz * xx) / det_y;
    } // det Z biggest
    else
    {
        if (det_z <= 0.0)
        {
            throw (-1);
        }

        dir_x = (yz * xy - xz * yy ) / det_z;
        dir_y = (xz * xy - yz * xx ) / det_z;
        dir_z = 1.0;
    }

    return POINT_T(dir_x, dir_y, dir_z).normalized();
}

} // namespace fastsense::tsdf
