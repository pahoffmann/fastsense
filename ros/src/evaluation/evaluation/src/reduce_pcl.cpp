#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

struct PCLReducer
{
    int height;
    int width;
    int points_per_row;
    
    void printDebug(const sensor_msgs::PointCloud2::ConstPtr& pcl)
    {
        points_per_row = pcl->row_step / pcl->point_step / height;
        ROS_DEBUG_STREAM("Width     : " << pcl->width);
        ROS_DEBUG_STREAM("Height    : " << pcl->height);
        ROS_DEBUG_STREAM("Point Step: " << pcl->point_step);
        ROS_DEBUG_STREAM("Row Step  : " << pcl->row_step);
        ROS_DEBUG_STREAM("H. Points : " << pcl->row_step / pcl->point_step);
        ROS_DEBUG_STREAM("Dims 2d   : " << points_per_row << "x" << height);
    }

    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl)
    {
        printDebug(pcl);

        ROS_ASSERT(pcl->row_step / pcl->point_step % heigth == 0);
        ROS_ASSERT(points_per_row == width);
        ROS_ASSERT(points_per_row * 128 == pcl->width * pcl->height);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reduce_pcl");
    ros::NodeHandle n("~");

    PCLReducer reducer;
    n.param("height", reducer.height, 128);
    n.param("width", reducer.width, 1024);

    ROS_ASSERT(reducer.height > 0 && reducer.width > 0);
    ROS_INFO_STREAM("Height: " << reducer.height);
    ROS_INFO_STREAM("Width : " << reducer.width);

    ros::Subscriber sub = n.subscribe("/points_raw", 10000, &PCLReducer::pclCallback, &reducer);
    ros::spin();

    return 0;
}
