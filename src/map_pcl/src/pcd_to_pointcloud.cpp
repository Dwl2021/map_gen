#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

std::string pcd_file_path, frame_id;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_to_pointcloud");
    ros::NodeHandle nh("~");
    // read the path to the PCD file from the launch file
    // read the frame_id
    nh.param("pcd_file_path", pcd_file_path, std::string("map.pcd"));
    nh.param("frame_id", frame_id, std::string("world_enu"));


    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Load PCD file using PCL
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", pcd_file_path.c_str());
        return (-1);
    }

    // Convert to ROS data type
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = frame_id;  // Set the appropriate frame ID

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
