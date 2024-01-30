#include "robot.h"


Robot::Robot( string namespace_,
              float radius_, 
              float max_rv_,
              float max_tv_,
              std::shared_ptr<World> w_, 
              const Pose& pose_)
    : radius(radius_), 
      WorldItem(w_, pose_), 
      tv(0.0), 
      rv(0.0),
      max_rv(max_rv_), 
      max_tv(max_tv_), 
      odom_pub(nh.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", QUEUE_SIZE)),
      cmd_vel_sub(nh.subscribe("/" + namespace_ + "/cmd_vel", QUEUE_SIZE, &Robot::cmdVelCallback, this)) {}

Robot::Robot( string namespace_,
              float radius_, 
              float max_rv_,
              float max_tv_,
              std::shared_ptr<WorldItem> parent_,
              const Pose& pose_)
    : radius(radius_), 
      WorldItem(parent_, pose_), 
      tv(0.0), 
      rv(0.0),
      max_rv(max_rv_), 
      max_tv(max_tv_), 
      odom_topic("/" + namespace_ + "/odom"),
      cmd_vel_topic("/" + namespace_ + "/cmd_vel"),
      odom_pub(nh.advertise<nav_msgs::Odometry>(odom_topic, QUEUE_SIZE)),
      cmd_vel_sub(nh.subscribe(cmd_vel_topic, QUEUE_SIZE, &Robot::cmdVelCallback, this)) {}

void Robot::draw() {
  int int_radius = radius * world->i_res;
  IntPoint p = world->world2grid(poseInWorld().translation());
  cv::circle(world->display_image, cv::Point(p.y(), p.x()), int_radius,
             cv::Scalar::all(0), -1);
}

void Robot::timeTick(float dt) {
  Pose motion = Pose::Identity();
  motion.translation() << tv * dt, 0;
  motion.rotate(rv * dt);

  Pose next_pose = pose_in_parent * motion;
  IntPoint ip = world->world2grid(next_pose.translation());
  int int_radius = radius * world->i_res;
  if (!world->collides(ip, int_radius)) pose_in_parent = next_pose;

  // Translational component extraction
  Eigen::Vector2f msg_translation = pose_in_parent.translation();
  float msg_x = msg_translation.x();
  float msg_y = msg_translation.y();

  // Rotational component extraction
  Eigen::Rotation2Df msg_rotation(pose_in_parent.linear());
  float msg_theta = msg_rotation.angle();

  // Publish odometry data
  mrsim::rodom odom;
  odom.x = msg_x;
  odom.y = msg_y;
  odom.theta = msg_theta;
  odom_pub.publish(odom);
}

void Robot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // float new_tv = msg->linear.x;
  // float new_rv = msg->angular.z; 
  if (msg->linear.x > max_tv) tv = max_tv;
  else tv = msg->linear.x;
  if (msg->angular.z > max_rv) rv = max_rv;
  else rv = msg->angular.z;
}

