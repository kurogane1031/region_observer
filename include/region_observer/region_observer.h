#ifndef REGION_OBSERVER_REGION_OBSERVER_H_
#define REGION_OBSERVER_REGION_OBSERVER_H_
#include <iostream>
#include <vector>
namespace perception{
enum class SignalColor{
  NONE = -1,
  RED = 1,
  YELLOW = 2,
  GREEN = 3,
};

enum class RegionType{
  INDOOR,
  OUTDOOR,
  CROSSWALK,
  TRAFFIC_LIGHT, // Separated for case like Crosswalk without traffic light etc.
  DOOR,
  AUTOMATED_DOOR,
  RegionType_size_t,
};

class RegionObserver{
 public:
  void checkVisited();
  void checkWithin();
  void setRegionOfInterest(const std::vector<unsigned int>& what_region,
                           const double &x_l, const double &y_l,
                           const double &x_u, const double &y_u);
  void updatePosition(const double& px, const double& py);
  bool isWithin();
  bool isVisited();
  std::vector<unsigned int> whatRegion();

 private:
  double pose_x {0.0};
  double pose_y {0.0};
  double x_lb {0.0}; //!< \brief region of interest lower bound x coordinate
  double y_lb {0.0}; //!< \brief region of interest lower bound y coordinate
  double x_ub {1.0}; //!< \brief region of interest upper bound x coordinate
  double y_ub {1.0}; //!< \brief region of interest upper bound y coordinate
  size_t n_region_type{static_cast<size_t>(RegionType::RegionType_size_t)};
  std::vector<unsigned int> region_type {std::vector<unsigned int>(n_region_type, 0)};

  std::pair<bool, int> is_within {false, 0}; //!< \brief checks whether host is in the region or not
  bool is_visited {false}; //!< \brief checks whether the region has been visited or not
};

}
#endif // REGION_OBSERVER_REGION_OBSERVER_H_
