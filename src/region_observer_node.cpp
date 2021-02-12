#include "region_observer/region_observer_node.h"
#include <unordered_map>
#include <string>
#include <fstream>
#include <sstream>
#include <ros/console.h>
namespace perception{
RegionObserverNode::RegionObserverNode()
{
  ROS_INFO_STREAM("Initializing");
  initForRos();
}

RegionObserverNode::~RegionObserverNode()
{
}

void RegionObserverNode::getCommand(){

}

unsigned int RegionObserverNode::accelCommand(){
  std::vector<unsigned int> reg {rgobs.whatRegion()};
  if(rgobs.isWithin() && (reg[static_cast<unsigned int>(RegionType::TRAFFIC_LIGHT)]))
     return 0;
  else
     return 10;
}

// void RegionObserverNode::processTrafficRegion(){
//   traffic_signal_sub = nh.subscribe("traffic", 10, &RegionObserverNode::callbackFromTrafficSignal, this);
// }
void RegionObserverNode::parseConfigFile(const std::string &filename){
  regions.region.clear();
  std::ifstream f{filename};
  std::string row {};
  std::vector<std::string> headers {};
  headers.reserve(5);
  std::getline(f, row);
  {
    std::istringstream ss{row};
    std::string header {};
    while(std::getline(ss, header, ','))
    {
      headers.emplace_back(header);
    }
  }
  itolab_senior_car_msgs::RegionObserver region;
  while(std::getline(f, row))
  {
    std::istringstream ss{row};
    std::vector<std::string> columns {};
    std::string column {};
    while(std::getline(ss, column,','))
    {
      columns.push_back(column);
    }

    region.lower_bound.point.x = std::stof(columns[1]);
    region.lower_bound.point.y = std::stof(columns[2]);
    region.upper_bound.point.x = std::stof(columns[3]);
    region.upper_bound.point.y = std::stof(columns[4]);
    region.is_within = false;
    region.is_within_flag = 0;
    region.is_visited = false;
    for(int i = 5; i < columns.size(); ++i)
    {
      region.region_type.push_back(std::stoi(columns[i]));
    }
    regions.region.push_back(region);
  }
  region_numbers = regions.region.size();
}

void RegionObserverNode::initForRos(){
  pose_sub = nh.subscribe("current_pose", 10, &RegionObserverNode::callbackFromCurrentPose, this);
  to_servo_pub = nh.advertise<itolab_senior_car_msgs::RegionObserver>("/region_information", 5);
}

void RegionObserverNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg){
  auto &px = msg->pose.position.x;
  auto &py = msg->pose.position.y;
  rgobs.updatePosition(px, py);
  rgobs.checkWithin();
  rgobs.checkVisited();

  reg.is_visited = rgobs.isVisited();
  reg.is_within = rgobs.isWithin();
  if(region_idx < region_numbers)
  {
    ROS_INFO("idx %ld, within %d, visited %d",region_idx, reg.is_within, reg.is_visited);
  }
  else
  {
    ROS_INFO("All regions visited. Yay!!!");
  }

  if(reg.is_visited == true && region_idx != region_numbers){
    ++region_idx;
    rgobs.resetFlags();
    reg = regions.region[region_idx];
    reg.is_visited = regions.region[region_idx].is_visited;
    reg.is_within = regions.region[region_idx].is_within;
    reg.is_within_flag = regions.region[region_idx].is_within_flag;
    rgobs.setRegionOfInterest(reg.region_type,
                              reg.lower_bound.point.x,
                              reg.lower_bound.point.y,
                              reg.upper_bound.point.x,
                              reg.upper_bound.point.y);
  }
}

void RegionObserverNode::callbackFromTrafficSignal(const int& msg){

}

void RegionObserverNode::callbackFromLidarDetection(const itolab_senior_car_msgs::DetectedObjectArrayConstPtr& msg){

}

void RegionObserverNode::run(){
  ROS_INFO_STREAM("START REGIONNNNNNN OBSERVEERRRRRR");
  parseConfigFile("/home/zulfaqar/develop/ros/catkin_ws/src/region_observer/param/regions.csv");
  reg = regions.region[region_idx];
  rgobs.setRegionOfInterest(reg.region_type,
                            reg.lower_bound.point.x,
                            reg.lower_bound.point.y,
                            reg.upper_bound.point.x,
                            reg.upper_bound.point.y);
  while(ros::ok()){
    ros::spinOnce();
  }
}
}
