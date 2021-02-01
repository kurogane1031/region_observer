#include "region_observer/region_observer_node.h"

namespace perception{
RegionObserverNode::RegionObserverNode()
{
  initForRos();
}

RegionObserverNode::~RegionObserverNode()
{
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
