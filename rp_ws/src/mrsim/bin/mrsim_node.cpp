#include <ros/ros.h>  
 
#include <sys/time.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <map>

#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "utils.h"

#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;
using namespace std;

int main(int argc, char** argv) {

  if (argc < 2) {
    cerr << "Error: no config.jsosn file provided:\n" 
    << "Please insert your configuration file in the project config directory.\n"
    << "Then pass the name of the file as arg to the node.\n" << endl;
    return 1;
  }

  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  const string ROOT_PATH = "./..";
  const string CONFIG_PATH = ROOT_PATH + "/config/" + argv[1];

  Json::Value root = readJson(CONFIG_PATH);
  string map_name = root["map"].asString();
  cout << "Map -> " << map_name << endl;
  string IMAGE_PATH = ROOT_PATH + "/map/" +  map_name;

  World world(42); 
  WorldPointer world_ptr(&world, [](World*){ });   
  world.loadFromImage(IMAGE_PATH);  

  int NUM_ROBOTS = 0;

  vector<shared_ptr<Robot>> robots; 
  vector<shared_ptr<Lidar>> lidars;
  map<int, shared_ptr<Robot>> id2robots;

  //BUILD ALL OBJECTS TO PUT IN THE WORLD

  for(const Json::Value& item: root["items"]) {

    const int id = item["id"].asInt();
    const  string type = item["type"].asString();
    // string frame_id = item["frame_id"].asString();
    const string namespace_ = item["namespace"].asString();

    double pose_x = item["pose"][0].asInt();
    double pose_y = item["pose"][1].asInt();
    double theta = item["pose"][2].asDouble();

    const int id_p = item["parent"].asInt(); 

    if (type == "robot") {
      // Robot parameters
      double radius = item["radius"].asDouble();
      float max_rv = item["max_rv"].asFloat();
      float max_tv = item["max_tv"].asFloat();
      
      Pose robot_pose = Pose::Identity();
      robot_pose.translation() = world_ptr->grid2world(Eigen::Vector2i(pose_x, pose_y));
      robot_pose.linear() = Eigen::Rotation2Df(theta).matrix();
      Robot* r;

      if (id_p != -1){
        shared_ptr<Robot> parent_ptr= id2robots[id_p]; 
        Pose parent_pose = parent_ptr->poseInWorld();
        r = new Robot(namespace_, radius, max_rv, max_tv, parent_ptr, parent_pose); 
      }
      else {
        r = new Robot(namespace_, radius, max_rv, max_tv, world_ptr, robot_pose);
      }
      shared_ptr<Robot> r_(r, [](Robot* r){ }); 
      id2robots[id] = r_; 
      robots.push_back(r_);
      NUM_ROBOTS++;
    }
    else {
      // Lidar parameters
      float fov = item["fov"].asFloat();
      double max_range_l = item["max_range"].asDouble();
      int num_beams_l = item["num_beams"].asInt();
  
      Pose lidar_pose = Pose::Identity();
      lidar_pose.translation() = world_ptr->grid2world(Eigen::Vector2i(pose_x, pose_y));
      lidar_pose.linear() = Eigen::Rotation2Df(theta).matrix();
      Lidar* l;

      if (id_p != -1){
        shared_ptr<Robot> parent_ptr= id2robots[id_p]; 
        l = new Lidar(namespace_, fov, max_range_l, num_beams_l, parent_ptr, lidar_pose);
      }
      else {
        l = new Lidar(namespace_, fov, max_range_l, num_beams_l, world_ptr, lidar_pose);
      }

      shared_ptr<Lidar> l_(l, [](Lidar* l){ }); 
      lidars.push_back(l_);
    }
  }
 
  
  world.draw();
  cv::waitKey(1);

  float delay = 0.1;
  ros::Rate loop_rate(10);

  cout << "Running primary node" << endl;

  while (ros::ok()) {

    world.draw();
    int k = cv::waitKey(1);
    if (k == 27) break;

    ros::spinOnce();
    world.timeTick(delay);
    this_thread::sleep_for(chrono::milliseconds(10)); // sleep for x milliseconds

  }

  cv::destroyAllWindows();
  return 0;
} 
