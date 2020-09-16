/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

using namespace fastsense::tsdf;

template<typename POINT_T, typename SET_T>
POINT_T AverageNormal<POINT_T, SET_T>::operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const
{
    size_t width = points[0].size();
    size_t max_ring = points.size() - 1;
    const auto& point = points[query_ring][query_index];

    POINT_T up(0, 0, 0);
    POINT_T down(0, 0, 0);
    POINT_T left = point;
    POINT_T right = point;
    int up_cnt = 0, down_cnt = 0, left_cnt = 1, right_cnt = 1;
    int index;
    for (int di = 0; di < neighbor_range_; di++)
    {
        if (di % 2 == 0)
        {
            index = (query_index + di / 2) % width;
        }
        else
        {
            index = (query_index + width - di / 2) % width;
        }
        if (query_ring < max_ring)
        {
            const auto& u = points[query_ring + 1][index];
            if (!std::isnan(u.x()) && (up_cnt == 0 || (u - (up / up_cnt)).squaredNorm() < max_dist_))
            {
                up += u;
                up_cnt++;
            }
        }
        if (query_ring > 0)
        {
            const auto& d = points[query_ring - 1][index];
            if (!std::isnan(d.x()) && (down_cnt == 0 || (d - (down / down_cnt)).squaredNorm() < max_dist_))
            {
                down += d;
                down_cnt++;
            }
        }
        index = (query_index + width - di - 1) % width;
        const auto& l = points[query_ring][index];
        if (!std::isnan(l.x()) && (l - (left / left_cnt)).squaredNorm() < max_dist_)
        {
            left += l;
            left_cnt++;
        }
        index = (query_index + di + 1) % width;
        const auto& r = points[query_ring][index];
        if (!std::isnan(r.x()) && (r - (right / right_cnt)).squaredNorm() < max_dist_)
        {
            right += r;
            right_cnt++;
        }
    }
    if (up_cnt == 0 && down_cnt != 0)
    {
        up = point;
        up_cnt = 1;
    }
    else if (down_cnt == 0)
    {
        down = point;
        down_cnt = 1;
    }

    if (up_cnt == 0 || down_cnt == 0 || (left_cnt == 1 && right_cnt == 1))
    {
        throw (-1);
    }
    return (right / right_cnt - left / left_cnt)
           .cross(up / up_cnt - down / down_cnt).normalized();
}