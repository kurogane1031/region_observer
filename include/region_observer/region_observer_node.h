#ifndef __REGION_OBSERVER_NODE_H_
#define __REGION_OBSERVER_NODE_H_

#include "region_observer/region_observer.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "itolab_senior_car_msgs/DetectedObjectArray.h"
#include "itolab_senior_car_msgs/RegionObserverArray.h"

#include <string>
namespace perception
{
  class RegionObserverNode
  {
    public:
      RegionObserverNode();
      ~RegionObserverNode();
      /* void run(); */
      friend struct RegionObserverNodeTest;
      RegionObserver rgobs {};
      void getCommand();
      unsigned int accelCommand();

      // void processTrafficRegion();
      void initForRos();
      void parseConfigFile(const std::string & filename);
      void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
      void callbackFromTrafficSignal(const int& msg);
      void callbackFromLidarDetection(const itolab_senior_car_msgs::DetectedObjectArrayConstPtr& msg);
      itolab_senior_car_msgs::RegionObserverArray regions;

    private:
      ros::NodeHandle nh;
      ros::Subscriber pose_sub;
      ros::Subscriber traffic_signal_sub;
      ros::Subscriber lidar_detection_sub;
      ros::Publisher to_servo_pub;

  };
}


#endif // __REGION_OBSERVER_NODE_H_
