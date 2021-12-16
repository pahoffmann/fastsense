namespace fastsense::bridge
{

struct EIGEN_ALIGN16 PointOuster
{
    PCL_ADD_POINT4D;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(fastsense::bridge::PointOuster,
    (float, x, x)
    (float, y, y)
    (float, z, z)
)