#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>

ros::Publisher pcl_pub;
sensor_msgs::PointCloud2 map_msg_;
pcl::PointCloud<pcl::PointXYZ> map_cloud_, last_map_cloud_;
ros::Subscriber click_sub_, undo_sub_, clear_sub_;
double length, width, height;
double step_length, step_width, step_height;
std::string frame_id;

void gen_pcl(double length, double width, double height, double step_length,
             double step_width, double step_height, Eigen::Vector3d origin,
             pcl::PointCloud<pcl::PointXYZ> &cloud);

void clickCallback(const geometry_msgs::PoseStamped &msg);
void undoCallback(const std_msgs::Empty &msg);
void clearCallback(const std_msgs::Empty &msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "click_map");
  ros::NodeHandle n("~");
  n.param("length", length, 1.0);
  n.param("width", width, 1.0);
  n.param("height", height, 1.0);
  n.param("step_length", step_length, 0.1);
  n.param("step_width", step_width, 0.1);
  n.param("step_height", step_height, 0.1);
  n.param("frame", frame_id, std::string("world"));

  pcl_pub = n.advertise<sensor_msgs::PointCloud2>("click_map", 1);
  click_sub_ = n.subscribe("/move_base_simple/goal", 10, clickCallback);
  undo_sub_ = n.subscribe("/undo", 10, undoCallback);
  clear_sub_ = n.subscribe("/clear", 10, clearCallback);
  
  while (ros::ok()) {
    pcl::toROSMsg(map_cloud_, map_msg_);
    map_msg_.header.frame_id = frame_id;
    pcl_pub.publish(map_msg_);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
}

void gen_pcl(double length, double width, double height, double step_length,
             double step_width, double step_height, Eigen::Vector3d origin,
             pcl::PointCloud<pcl::PointXYZ> &cloud) {
  pcl::PointXYZ pt;
  int radio = 0.3;
  int num_steps_length = static_cast<int>(std::ceil(length / step_length));
  int num_steps_width = static_cast<int>(std::ceil(width / step_width));
  int num_steps_height = static_cast<int>(std::ceil(height / step_height));

  for (int i = -num_steps_length / 2; i <= num_steps_length / 2; ++i) {
    for (int j = -num_steps_width / 2; j <= num_steps_width / 2; ++j) {
      for (int k = 0; k <= num_steps_height; ++k) {
        if (i > num_steps_length * radio - num_steps_length / 2 &&
            i < num_steps_length * (1 - radio) - num_steps_length / 2 &&
            j > num_steps_width * radio - num_steps_length / 2 &&
            j < num_steps_width * (1 - radio) - num_steps_length / 2 &&
            k > num_steps_height * radio &&
            k < num_steps_height * (1 - radio)) {
          continue;
        }
        pt.x = origin[0] + i * step_length;
        pt.y = origin[1] + j * step_width;
        pt.z = origin[2] + k * step_height;
        cloud.push_back(pt);
      }
    }
  }
}

void clickCallback(const geometry_msgs::PoseStamped &msg) {
  Eigen::Vector3d origin(msg.pose.position.x, msg.pose.position.y,
                         msg.pose.position.z);

  pcl::PointCloud<pcl::PointXYZ> temp_cloud;
  last_map_cloud_ = map_cloud_;
  gen_pcl(length, width, height, step_length, step_width, step_height, origin,
          temp_cloud);
  map_cloud_ += temp_cloud;
  map_cloud_.width = map_cloud_.points.size();
  map_cloud_.height = 1;
  map_cloud_.is_dense = true;
  pcl::toROSMsg(map_cloud_, map_msg_);
  map_msg_.header.frame_id = frame_id;
  pcl_pub.publish(map_msg_);
}

void undoCallback(const std_msgs::Empty &msg) {
  ROS_WARN("Undo last operation");
  // Implement undo functionality if required
  map_cloud_ = last_map_cloud_;
  map_cloud_.width = map_cloud_.points.size();
  map_cloud_.height = 1;
  map_cloud_.is_dense = true;
  pcl::toROSMsg(map_cloud_, map_msg_);
  map_msg_.header.frame_id = frame_id;
  pcl_pub.publish(map_msg_);
}

void clearCallback(const std_msgs::Empty &msg) {
  ROS_WARN("Clearing map");
  map_cloud_.clear();
  map_cloud_.width = map_cloud_.points.size();
  map_cloud_.height = 1;
  map_cloud_.is_dense = true;
  pcl::toROSMsg(map_cloud_, map_msg_);
  map_msg_.header.frame_id = frame_id;
  pcl_pub.publish(map_msg_);
}
