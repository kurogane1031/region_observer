#ifndef REGION_OBSERVER_REGION_OBSERVER_H_
#define REGION_OBSERVER_REGION_OBSERVER_H_
#include <ros/ros.h>
#include <iostream>

namespace perception{
enum class SignalColor : int{
  NONE = -1,
  RED = 1,
  YELLOW = 2,
  GREEN = 3,
};

class RegionObserver{
 public:
  void checkVisited();
  void checkWithin();
  void setRegionOfInterest(const double &x_l, const double &y_l,
                           const double &x_u, const double &y_u);
  void updatePosition(const double& px, const double& py);
  bool isWithin();
  bool isVisited();
 private:
  double pose_x {0.0};
  double pose_y {0.0};
  double x_lb {0.0}; //!< \brief region of interest lower bound x coordinate
  double y_lb {0.0}; //!< \brief region of interest lower bound y coordinate
  double x_ub {1.0}; //!< \brief region of interest upper bound x coordinate
  double y_ub {1.0}; //!< \brief region of interest upper bound y coordinate

  std::pair<bool, int> is_within {false, 0}; //!< \brief checks whether host is in the region or not
  bool is_visited {false}; //!< \brief checks whether the region has been visited or not
};

class RegionObserverNode{
  ros::Subscriber pose_sub {};
};
}
#endif // REGION_OBSERVER_REGION_OBSERVER_H_
