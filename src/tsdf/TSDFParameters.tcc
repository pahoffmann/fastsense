using namespace fastsense::tsdf;

template<typename SCALAR_T, typename VEC_T>
void TSDFParameters<SCALAR_T, VEC_T>::load(const ros::NodeHandle& n)
{
    SCALAR_T map_size_x = map_size_.x();
    SCALAR_T map_size_y = map_size_.y();
    SCALAR_T map_size_z = map_size_.z();

    if(n.hasParam("max_distance"))
    {   
        n.getParam("max_distance", max_distance_);
    }

    if(n.hasParam("map_resolution"))
    {
        n.getParam("map_resolution", map_resolution_);
    }

    if(n.hasParam("map_size/x"))
    {
        n.getParam("map_size/x", map_size_x);
    }

    if(n.hasParam("map_size/y"))
    {
        n.getParam("map_size/y", map_size_y);
    }

    if(n.hasParam("map_size/z"))
    {
        n.getParam("map_size/z", map_size_z);
    }

    setMapSize(VEC_T(map_size_x, map_size_y, map_size_z));
}

template<typename SCALAR_T, typename VEC_T>
void TSDFParameters<SCALAR_T, VEC_T>::updateMap()
{
    scanner_pos_ = map_size_ / 2 / map_resolution_;

    map_grid_size_x_ = map_size_.x() / map_resolution_;
    map_grid_size_y_ = map_size_.y() / map_resolution_;
    map_grid_size_z_ = map_size_.z() / map_resolution_;

    total_size_ = map_grid_size_x_ * map_grid_size_y_ * map_grid_size_z_;
}

template<typename SCALAR_T, typename VEC_T>
std::ostream& operator<<(std::ostream& os, const TSDFParameters<SCALAR_T, VEC_T>& params)
{
    os << "Parameters:\n"
       << "\ttau: "             << params.tau_             << '\n'
       << "\tmap_resolution: "  << params.map_resolution_  << '\n'
       << "\tmap_size_x: "      << params.map_size_.x()    << '\n'
       << "\tmap_size_y: "      << params.map_size_.y()    << '\n'
       << "\tmap_size_z: "      << params.map_size_.z()    << '\n'
       << "\tscanner_pos_x: "   << params.scanner_pos_.x() << '\n'
       << "\tscanner_pos_y: "   << params.scanner_pos_.y() << '\n'
       << "\tscanner_pos_z: "   << params.scanner_pos_.z() << '\n'       
       << "\tmap_grid_size_x: " << params.map_grid_size_x_ << '\n'
       << "\tmap_grid_size_y: " << params.map_grid_size_y_ << '\n'
       << "\tmap_grid_size_z: " << params.map_grid_size_z_ << '\n'
       << "\ttotal_size: "       << params.total_size_      << std::endl;
}