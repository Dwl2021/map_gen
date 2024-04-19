#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>
#include <cmath>
using namespace std;

ros::Publisher pcl_pub;
sensor_msgs::PointCloud2 map_msg_;
pcl::PointCloud<pcl::PointXYZ> map_cloud_;

ros::Subscriber click_sub_;
vector<Eigen::Vector3d> vec_points;
double width, height;
;
double step_length, step_height, step_width;
std::string frame_id;
double x, y;

void gen_pcl(double length, double width, double height, double step_length,
                  double step_width, double step_height, Eigen::Vector3d p1,
                  Eigen::Vector3d dir1, Eigen::Vector3d dir2, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  pcl::PointXYZ pt;
  int num_steps_length = static_cast<int>(std::ceil(length / step_length));
  int num_steps_width = static_cast<int>(std::ceil(width / step_width));
  int num_steps_height = static_cast<int>(std::ceil(height / step_height));

  for (int i = 0; i <= num_steps_length; ++i){
    double l1 = std::min(i * step_length, length);
    Eigen::Vector3d tmp1 = p1 + l1 * dir1;

    for (int j = -num_steps_width / 2; j <= num_steps_width / 2; ++j){
      double l2 = std::min(j * step_width, width);
      Eigen::Vector3d tmp2 = tmp1 + l2 * dir2;

      for (int k = -1e-3; k <= num_steps_height; ++k){
        double h = std::min(k * step_height, height);
        pt.x = tmp2[0];
        pt.y = tmp2[1];
        pt.z = h;
        cloud.push_back(pt);
      }
    }
  }
}

void clickCallback(const geometry_msgs::PoseStamped &msg){
  x = msg.pose.position.x;
  y = msg.pose.position.y;
  vec_points.push_back(Eigen::Vector3d(x, y, 0));
  if (vec_points.size() < 2)
    return;

  // Generate wall using two points
  Eigen::Vector3d p1 = vec_points[0];
  Eigen::Vector3d p2 = vec_points[1];
  vec_points.clear();

  Eigen::Vector3d dir1 = (p2 - p1).normalized();
  double length = (p2 - p1).norm();
  Eigen::Vector3d dir2;
  dir2[0] = -dir1[1];
  dir2[1] = dir1[0];
  gen_pcl(length, width, height, step_length, step_width, \
                step_height, p1, dir1, dir2, map_cloud_);
  map_cloud_.width = map_cloud_.points.size();
  map_cloud_.height = 1;
  map_cloud_.is_dense = true;
  pcl::toROSMsg(map_cloud_, map_msg_);
  map_msg_.header.frame_id = frame_id;
  pcl_pub.publish(map_msg_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "click_map");
  ros::NodeHandle n("~");
  n.param("width", width, 0.15);
  n.param("height", height, 2.5);
  n.param("step_length", step_length, 0.1);
  n.param("step_height", step_height, 0.1);
  n.param("step_width", step_width, 0.1);
  n.param("frame", frame_id, std::string("world"));

  pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/click_map", 1);
  click_sub_ = n.subscribe("/move_base_simple/goal", 10, clickCallback);
  ros::Duration(0.5).sleep();

  while (ros::ok())
  {
    pcl::toROSMsg(map_cloud_, map_msg_);
    map_msg_.header.frame_id = frame_id;
    pcl_pub.publish(map_msg_);

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
}
