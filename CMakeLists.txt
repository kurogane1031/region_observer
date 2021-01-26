cmake_minimum_required(VERSION 3.18.0)
project(region_observer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rostest
  rosunit
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES region_observer
#  CATKIN_DEPENDS roscopp roslint rostest rosunit std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(region
  src/region_observer.cpp
  src/region_observer_node.cpp
  )
target_compile_features(region
  PUBLIC
      cxx_std_14
  )
target_include_directories(region
  PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(region
  ${catkin_LIBRARIES})
add_dependencies(region
  ${catkin_EXPORTED_TARGETS})

install(
  TARGETS region
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test-region
    test/test_region.test
    test/src/test_region.cpp
    )
  add_dependencies(test-region ${catkin_EXPORTED_TARGETS})
  target_include_directories(test-region
    PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    )
  target_compile_features(test-region
    PRIVATE
    cxx_std_14)
  target_link_libraries(test-region ${catkin_LIBRARIES})
endif()
