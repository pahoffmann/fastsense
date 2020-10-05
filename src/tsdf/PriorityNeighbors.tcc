#pragma once

/**
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <set>
#include <queue>

namespace fastsense::tsdf
{

template<typename POINT_T, typename SET_T>
std::vector<POINT_T> PriorityNeighbors<POINT_T, SET_T>::operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const
{
    const auto& query_point = points[query_ring][query_index];

    std::priority_queue<std::pair<double, std::pair<short, unsigned int>>, std::vector<std::pair<double, std::pair<short, unsigned int>>>, PairComparator> queue;
    std::set<std::pair<short, unsigned int>, PairComparator> neighbor_set;

    std::vector<POINT_T> result_points(number_of_neighbors_);

    queue.push(std::make_pair(0.0, std::make_pair(query_ring, query_index)));
    neighbor_set.insert(std::make_pair(query_ring, query_index));

    auto i = 0u;

    for (; i < number_of_neighbors_ && queue.size() != 0; ++i)
    {
        const auto next = queue.top();
        queue.pop();

        result_points[i] = points[next.second.first][next.second.second];

        // Right

        auto ring = next.second.first;
        auto index = (next.second.second + 1) % points[ring].size();

        while (std::isnan(points[ring][index].x()) && index != next.second.second)
        {
            index = (index + 1) % points[ring].size();
        }

        auto paired = std::make_pair(ring, index);

        if (neighbor_set.find(paired) == neighbor_set.end())
        {
            neighbor_set.insert(paired);
            queue.push(std::make_pair((query_point - points[ring][index]).squaredNorm(), paired));
        }

        // Left

        ring = next.second.first;
        index = (next.second.second + points[ring].size() - 1) % points[ring].size();

        while (std::isnan(points[ring][index].x()) && index != next.second.second)
        {
            index = (index + points[ring].size() - 1) % points[ring].size();
        }

        paired = std::make_pair(ring, index);

        if (neighbor_set.find(paired) == neighbor_set.end())
        {
            neighbor_set.insert(paired);
            queue.push(std::make_pair((query_point - points[ring][index]).squaredNorm(), paired));
        }

        // Up

        index = next.second.second;
        ring = (next.second.first + 1) % points.size();

        while (std::isnan(points[ring][index].x()) && ring != next.second.first)
        {
            ring = (ring + 1) % points.size();
        }

        paired = std::make_pair(ring, index);

        if (neighbor_set.find(paired) == neighbor_set.end())
        {
            neighbor_set.insert(paired);
            queue.push(std::make_pair((query_point - points[ring][index]).squaredNorm(), paired));
        }

        // Down

        index = next.second.second;
        ring = (next.second.first + points.size() - 1) % points.size();

        while (std::isnan(points[ring][index].x()) && ring != next.second.first)
        {
            ring = (ring + points.size() - 1) % points.size();
        }

        paired = std::make_pair(ring, index);

        if (neighbor_set.find(paired) == neighbor_set.end())
        {
            neighbor_set.insert(paired);
            queue.push(std::make_pair((query_point - points[ring][index]).squaredNorm(), paired));
        }
    }

    result_points.resize(i);
    return result_points;
}

} // namespace fastsense::tsdf
