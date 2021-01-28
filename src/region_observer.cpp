#include "region_observer/region_observer.h"

namespace perception{
void RegionObserver::checkVisited(){
  if( (is_within.first == false) &&
      (is_within.second == 2))
  {
    is_visited = true;
  }
}

void RegionObserver::checkWithin(){
  if( (((pose_x < x_lb) && (pose_y < y_lb))  ||
       ((pose_x > x_ub) && (pose_y > y_ub))) &&
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

void RegionObserver::setRegionOfInterest(const double &x_l, const double &y_l,
                                         const double &x_u, const double &y_u){
  x_lb = x_l;
  y_lb = y_l;
  x_ub = x_u;
  y_ub = y_u;
}

bool RegionObserver::isWithin(){
  return is_within.first;
}
bool RegionObserver::isVisited()
{
  return is_visited;
}
void RegionObserver::updatePosition(const double& px, const double& py)
{
  pose_x = px;
  pose_y = py;
}
}
