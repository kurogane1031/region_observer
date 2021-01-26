#include "region_observer/region_observer.h"
#include <boost/version.hpp>
#include <iostream>
int main(){
  Region region;
  region.printVar();
  std::cout << "Boost version: " << BOOST_LIB_VERSION << "\n";
}
