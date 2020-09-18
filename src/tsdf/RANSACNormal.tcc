#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

#include <random>
#include <set>
#include <iostream>

namespace fastsense::tsdf
{

template<typename POINT_T, typename SET_T>
POINT_T RANSACNormal<POINT_T, SET_T>::operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const
{
    auto neighbors = (*(this->neighbors_))(query_ring, query_index, points);

    size_t numPoints = neighbors.size();

    if (numPoints == 0)
    {
        throw (-1);
    }

    POINT_T bestNorm(0, 0, 1);

    float bestdist = std::numeric_limits<float>::max();

    int iterations              = 0;
    int nonimproving_iterations = 0;

    std::default_random_engine generator;
    std::uniform_int_distribution<unsigned long> distribution(0, numPoints - 1);
    auto number = std::bind(distribution, generator);

    while ((nonimproving_iterations < max_nonimpiter_) && (iterations < max_iterations_))
    {
        // randomly choose 3 disjoint points
        int c = 0;

        std::set<unsigned long> ids;
        do
        {
            ids.insert(number());
            c++;
        }
        while (ids.size() < 3 && c <= 30);

        if (ids.size() < 3)
        {
            std::cout << "Deadlock" << std::endl;
            throw (-1);
        }

        auto id_iter = ids.begin();

        const auto& point1 = neighbors[*id_iter];
        const auto& point2 = neighbors[*(++id_iter)];
        const auto& point3 = neighbors[*(++id_iter)];

        auto normal = (point1 - point2).cross(point1 - point3).normalized();
        float p1Dist = point1.dot(normal);

        //compute error to at most 50 other randomly chosen points
        float dist = 0;
        int n = std::min(size_t{50}, numPoints);
        for (int i = 0; i < n; i++)
        {
            dist += fabs(neighbors[number()].dot(normal) - p1Dist);
        }
        if (n != 0)
        {
            dist /= n;
        }

        //a new optimum is found
        if (dist < bestdist)
        {
            bestdist = dist;
            bestNorm = normal;

            nonimproving_iterations = 0;
        }
        else
        {
            nonimproving_iterations++;
        }

        iterations++;
    }

    return bestNorm;
}

} // namespace fastsense::tsdf
