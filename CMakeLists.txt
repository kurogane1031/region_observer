cmake_minimum_required(VERSION 3.18.0)
project(region_observer)

find_package(catkin REQUIRED COMPONENTS
  itolab_senior_car_msgs
  roscpp
  roslint
  rostest
  rosunit
  std_msgs
  )
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")
set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")

option(USE_CONAN "Use Conan package manager to fetch dependencies" OFF)
if(USE_CONAN)
  include(${PROJECT_SOURCE_DIR}/cmake/conan.cmake)
  conan_check(REQUIRED)
  conan_cmake_run(
    REQUIRES boost/1.74.0
    UPDATE
    BASIC_SETUP CMAKE_TARGETS
    NO_OUTPUT_DIRS
    SETTINGS compiler.libcxx=libstdc++11
    BUILD missing
    )
else()
  find_package(Boost REQUIRED)
endif()
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
 itolab_senior_car_msgs
)

add_executable(region
  src/region_observer.cpp
  src/region_observer_node.cpp
  src/region_observer_main.cpp
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

if(USE_CONAN)
target_link_libraries(region
  PUBLIC
  ${catkin_LIBRARIES}
  CONAN_PKG::boost)
else()
target_link_libraries(region
  PUBLIC
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})
endif()

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

install(
  DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  )
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test-region
    test/test_region.test
    test/src/test_region.cpp
    src/region_observer.cpp
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

  add_rostest_gtest(test-region-node
    test/test_region_node.test
    test/src/test_region_node.cpp
    src/region_observer.cpp
    src/region_observer_node.cpp
    )
  add_dependencies(test-region-node ${catkin_EXPORTED_TARGETS})
  target_include_directories(test-region-node
    PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    )
  target_compile_options(test-region-node
    PRIVATE
    $<$<CXX_COMPILER_ID:Clang>:-Wall>
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    )
  target_compile_features(test-region-node
    PRIVATE
    cxx_std_14)
  target_link_libraries(test-region-node ${catkin_LIBRARIES})
endif()
