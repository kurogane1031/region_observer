#include "region_observer/region_observer_node.h"

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

void RegionObserverNode::initForRos(){
  pose_sub = nh.subscribe("current_pose", 10, &RegionObserverNode::callbackFromCurrentPose, this);
}

void RegionObserverNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg){

}

void RegionObserverNode::callbackFromTrafficSignal(const int& msg){

}

void RegionObserverNode::callbackFromLidarDetection(const itolab_senior_car_msgs::DetectedObjectArrayConstPtr& msg){

}
}
