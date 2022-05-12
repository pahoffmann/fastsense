#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <fstream>

ros::Publisher cloud_pub;

namespace fs = boost::filesystem;

struct MyPoint {
    float x;
    float y;
    float z;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_to_ros");
    ros::NodeHandle n("~");

    std::string dir;
    double time_rate;

    if (!n.getParam("dir", dir))
    {
        std::cout << "Please enter a directory as argument" << std::endl;
        return 0;
    }

    auto base_path = fs::path(dir);
    if (!fs::is_directory(base_path))
    {
        ROS_ERROR_STREAM("Entered directory doesn't exist!");
        return 0;
    }


    if (!n.getParam("rate", time_rate))
    {
        time_rate = 1.0;
    }

    ROS_INFO_STREAM("Starting with rate " << time_rate);

    std::ifstream time_stream(base_path / "times.txt");

    if(!time_stream)
    {
        ROS_ERROR_STREAM("Please enter a valid KITTI dataset folder with a \"time.txt\" in it!");
        return 0;
    }

    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);

    sensor_msgs::PointCloud2 cloud;

    cloud.header.frame_id = "base_link";
    cloud.height = 1;
    
    // Define point structure

    // Describe your custom MyPoint object
    cloud.fields.resize(3);
    // MyPoint.x
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[0].count = 1;
    // MyPoint.y
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[1].count = 1;
    // MyPoint.z
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    int count = 0;

    double last_stamp = 0;
    double stamp = 0;

    while(ros::ok() && count < 10000)
    {
        // load point cloud
        FILE *stream;

        std::stringstream binfile_name;
        binfile_name << std::setw(6) << std::setfill('0') << count;
        binfile_name << std::string(".bin");
        

        fs::path binfile_path = base_path / "velodyne" / binfile_name.str();

        if(!(time_stream >> stamp))
        {
            break;
        }

        stream = fopen(binfile_path.c_str(),"rb");

        if (stream == NULL)
        {
            ROS_ERROR_STREAM("Failed to load: " << binfile_path);
            break;
        }

        std::cout << "Loaded: " << binfile_path << std::endl;


        // allocate 4 MB buffer (only ~130*4*4 KB are needed)
        int32_t num = 1000000;

        float *data = (float*)malloc(num*sizeof(float));

        float *px = data+0;
        float *py = data+1;
        float *pz = data+2;
        float *pr = data+3;

        num = fread(data,sizeof(float),num,stream)/4;

        cloud.data.clear();

        cloud.width = num;
        cloud.point_step = sizeof(MyPoint);
        cloud.row_step = cloud.width * cloud.point_step;
        cloud.data.resize(cloud.row_step * cloud.height);

        // interpret the data as MyPoint vector
        MyPoint* pc2_points = reinterpret_cast<MyPoint*>(cloud.data.data());


        for (int32_t i=0; i<num; i++) 
        {   
            MyPoint p;
            p.x = *px;
            p.y = *py;
            p.z = *pz;

            pc2_points[i] = p;

            px+=4; py+=4; pz+=4; pr+=4;
        }

        fclose(stream);
        delete[] data;

        auto time_diff = (stamp - last_stamp) / time_rate;
        auto duration = ros::Duration(time_diff);
        duration.sleep();

        cloud.header.stamp = ros::Time(stamp);
        cloud_pub.publish(cloud);

        double hz;

        if (time_diff != 0.0)
        {
            hz = 1.0 / time_diff;
        }
        else
        {
            hz = 0.0;
        }

        ROS_INFO_STREAM(hz << "hz");


        last_stamp = stamp;
        ++count;
    }

    ROS_INFO("KITTI dataset has ended");

    return 0;
}
