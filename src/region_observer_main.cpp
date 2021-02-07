#include "region_observer/region_observer_node.h"
#include <boost/version.hpp>
#include <iostream>

int main(int argc, char** argv){
  ros::init(argc, argv, "region_observer");
  perception::RegionObserverNode ron;
  ron.run();
}
