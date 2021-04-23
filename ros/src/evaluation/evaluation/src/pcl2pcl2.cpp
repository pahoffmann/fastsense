#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

struct PCL2PCL2
{   
    /**
     * @brief Construct a new PCL2PCL2 object
     * 
     * @param n Nodehandle
     * @param topic topic to publish to
     */
    PCL2PCL2(ros::Publisher& pub) : pcl2pub{pub} {}
    
    /// default destructor
    ~PCL2PCL2() = default;

    /**
     * @brief Convert PointCloud to PointCloud2
     * 
     * @param pcl incoming PointCloud
     */
    void pclCallback(const sensor_msgs::PointCloud::ConstPtr& input)
    {
        // convert pcl to pcl2 manually, as sensor_msgs::convertPointCloudToPointCloud2 seems to be fishy
        // see: http://docs.ros.org/en/api/sensor_msgs/html/point__cloud__conversion_8h_source.html, l93
        // is dense is set to false automatically -> loam assumes pointcloud contains nan points

        sensor_msgs::PointCloud2 output;
        output.header = input->header;
        output.width  = input->points.size ();
        output.height = 1;
        output.fields.resize (3 + input->channels.size ());

        // Convert x/y/z to fields
        output.fields[0].name = "x"; output.fields[1].name = "y"; output.fields[2].name = "z";

        int offset = 0;
        // All offsets are *4, as all field data types are float32
        for (size_t d = 0; d < output.fields.size (); ++d, offset += 4)
        {
            output.fields[d].offset = offset;
            output.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
            output.fields[d].count  = 1;
        }
        output.point_step = offset;
        output.row_step   = output.point_step * output.width;
        
        // Convert the remaining of the channels to fields
        for (size_t d = 0; d < input->channels.size (); ++d)
        {
            output.fields[3 + d].name = input->channels[d].name;
        }

        output.data.resize (input->points.size () * output.point_step);
        output.is_bigendian = false;
        output.is_dense     = true;
        
        // Copy the data points
        for (size_t cp = 0; cp < input->points.size (); ++cp)
        {
            memcpy (&output.data[cp * output.point_step + output.fields[0].offset], &input->points[cp].x, sizeof (float));
            memcpy (&output.data[cp * output.point_step + output.fields[1].offset], &input->points[cp].y, sizeof (float));
            memcpy (&output.data[cp * output.point_step + output.fields[2].offset], &input->points[cp].z, sizeof (float));
            
            for (size_t d = 0; d < input->channels.size (); ++d)
            {
                if (input->channels[d].values.size() == input->points.size())
                {
                    memcpy (&output.data[cp * output.point_step + output.fields[3 + d].offset], &input->channels[d].values[cp], sizeof (float));
                }
            }
        }

        // publish the generated pcl
        pcl2pub.publish(output);
    }

    // publishes PointCloud2
    ros::Publisher& pcl2pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl2pcl2");
    ros::NodeHandle n("~");

    std::string intopic;
    n.param("intopic", intopic, std::string("pcl"));

    std::string outtopic;
    n.param("outtopic", outtopic, std::string("pcl2"));

    ROS_INFO_STREAM("Publishing pcl from '" << intopic << "' as pcl2 on '" << outtopic << "'.");

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(outtopic, 1000);
    PCL2PCL2 pcl2pcl2(pub);
    ros::Subscriber sub = n.subscribe(intopic, 1000, &PCL2PCL2::pclCallback, &pcl2pcl2);

    ros::spin();

    return 0;
}