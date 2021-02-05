#include "region_observer/region_observer_node.h"
#include <unordered_map>
#include <string>
#include <fstream>
#include <sstream>
namespace perception{
RegionObserverNode::RegionObserverNode()
{
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
}

void RegionObserverNode::initForRos(){
  pose_sub = nh.subscribe("current_pose", 10, &RegionObserverNode::callbackFromCurrentPose, this); parseConfigFile("/home/zulfaqar/develop/ros/catkin_ws/src/region_observer/param/regions.csv");
}

void RegionObserverNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg){

}

void RegionObserverNode::callbackFromTrafficSignal(const int& msg){

}

void RegionObserverNode::callbackFromLidarDetection(const itolab_senior_car_msgs::DetectedObjectArrayConstPtr& msg){

}
}
