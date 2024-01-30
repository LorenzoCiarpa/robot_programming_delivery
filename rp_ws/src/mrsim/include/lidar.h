#pragma once

#include "types.h"
#include "world.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl-1.10/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h> 


class Lidar : public WorldItem {
 public:
  Lidar(
      std::string namespace_, 
      float fov_, 
      float max_range_, 
      int num_beams_, 
      std::shared_ptr<World> w,
      const Pose& pose_ = Pose::Identity()
      );

  Lidar(
      std::string namespace_, 
      float fov_, 
      float max_range_, 
      int num_beams_,
      std::shared_ptr<WorldItem> p_, 
      const Pose& pose_ = Pose::Identity());

  void timeTick(float dt) override;

  void draw() override;

  float fov, max_range;
  int num_beams;
  std::vector<float> ranges;

  // ROS
  ros::NodeHandle node_handler;  // ROS Node Handle
  ros::Publisher scan_publisher;  // Publisher to send odometry data
};