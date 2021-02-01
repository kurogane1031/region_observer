#include "region_observer/region_observer_node.h"
#include <boost/version.hpp>
#include <iostream>

int main(){
  perception::RegionObserver region;
  std::cout << "Boost version: " << BOOST_LIB_VERSION << "\n";
}
