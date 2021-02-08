#include "region_observer/region_observer.h"

namespace perception{
void RegionObserver::checkVisited(){
  if( (is_within.first == false) &&
      (is_within.second == 2))
  {
    is_visited = true;
  }
}

std::vector<unsigned int> RegionObserver::whatRegion(){
  return region_type;
}

void RegionObserver::checkWithin(){
  if((!((pose_x >= x_lb) && (pose_y >= y_lb)   &&
        (pose_x <= x_ub) && (pose_y <= y_ub))) &&
      is_within.second == 1)
  {
    is_within.first = false;
    is_within.second = 2;
  }

  if( (pose_x >= x_lb) && (pose_y >= y_lb) &&
      (pose_x <= x_ub) && (pose_y <= y_ub) &&
      is_within.second == 0)
  {
    is_within.first = true;
    is_within.second = 1;
  }
}

void RegionObserver::setRegionOfInterest(const std::vector<unsigned int>& what_region,
                                         const double &x_l, const double &y_l,
                                         const double &x_u, const double &y_u)
{
  region_type = what_region,
  x_lb = x_l;
  y_lb = y_l;
  x_ub = x_u;
  y_ub = y_u;
  if(x_lb > x_ub)
  {
    std::swap(x_lb, x_ub);
  }
  if(y_lb > y_ub)
  {
    std::swap(y_lb, y_ub);
  }
}

bool RegionObserver::isWithin()
{
  return is_within.first;
}
bool RegionObserver::isVisited()
{
  return is_visited;
}

int RegionObserver::isWithinFlag()
{
  return is_within.second;
}

void RegionObserver::resetFlags()
{
  is_within.first = false;
  is_within.second = 0;
  is_visited = false;
}
void RegionObserver::updatePosition(const double& px, const double& py)
{
  pose_x = px;
  pose_y = py;
}
}
