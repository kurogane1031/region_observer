#include "region_observer/region_observer_node.h"
#include <gtest/gtest.h>
#include <geometry_msgs/PoseStamped.h>

struct RegionObserverNodeTest : public testing::Test{
  std::unique_ptr<perception::RegionObserverNode> observer {nullptr};
  geometry_msgs::PoseStamped position {};
  double & px = position.pose.position.x;
  double & py = position.pose.position.y;
  virtual void SetUp(){
    observer = std::make_unique<perception::RegionObserverNode>();
    std::vector<unsigned int> region_type {0,1,1,1,0,0}; // outdoor, crosswalk, traffic light

    observer->rgobs.setRegionOfInterest(region_type, 1.0, 1.0, 3.0, 3.0);
    position.pose.position.x = 1.5;
    position.pose.position.y = 1.5;
  }
  virtual void TearDown(){}

  RegionObserverNodeTest(){};
  ~RegionObserverNodeTest(){};
};

TEST_F(RegionObserverNodeTest, checkIfCurrentPoseIsWithinRegion){

  observer->rgobs.updatePosition(px, py);
  observer->rgobs.checkWithin();
  EXPECT_EQ(observer->rgobs.isWithin(), true);

  observer->rgobs.checkVisited();
  EXPECT_EQ(observer->rgobs.isVisited(), false);
}

TEST_F(RegionObserverNodeTest, checkIfVisited){
  // auto & px = position.pose.position.x;
  // auto & py = position.pose.position.y;
  observer->rgobs.updatePosition(px, py);
  observer->rgobs.checkWithin();
  EXPECT_EQ(observer->rgobs.isWithin(), true);

  observer->rgobs.checkVisited();
  EXPECT_EQ(observer->rgobs.isVisited(), false);

  // Update position. Move to the right
  position.pose.position.x = 4.0;
  position.pose.position.y = 1.5;
  observer->rgobs.updatePosition(px, py);
  observer->rgobs.checkWithin();
  EXPECT_EQ(observer->rgobs.isWithin(), false);

  observer->rgobs.checkVisited();
  EXPECT_EQ(observer->rgobs.isVisited(), true);
}

TEST_F(RegionObserverNodeTest, isWithinAndGetSensorData){
  observer->rgobs.updatePosition(px, py);
  observer->rgobs.checkWithin();
  EXPECT_EQ(observer->rgobs.isWithin(), true);

  // region observer class should only tell what region the host is currently in.
  // whatever commands should be handle by ros node
  // get traffic light information and publish suitable command
  observer->getCommand();
  EXPECT_EQ(observer->accelCommand(), 0);
}

TEST_F(RegionObserverNodeTest, parseAndCheck){
  std::string file{"/home/zulfaqar/develop/ros/catkin_ws/src/region_observer/param/regions.csv"};
  observer->parseConfigFile(file);
  itolab_senior_car_msgs::RegionObserver reg = observer->regions.region[0];
  ASSERT_FLOAT_EQ(reg.lower_bound.point.x, 13.943108);
  ASSERT_FLOAT_EQ(reg.lower_bound.point.y, -1.334371);

  observer->rgobs.setRegionOfInterest(reg.region_type,
                                      reg.lower_bound.point.x,
                                      reg.lower_bound.point.y,
                                      reg.upper_bound.point.x,
                                      reg.upper_bound.point.y);

  position.pose.position.x = 1.862637;
  position.pose.position.y = -0.778334;

  observer->rgobs.updatePosition(px, py);
  observer->rgobs.checkWithin();
  observer->rgobs.checkVisited();
  EXPECT_EQ(observer->rgobs.isWithin(), false);
  EXPECT_EQ(observer->rgobs.isVisited(), false);
  EXPECT_EQ(observer->rgobs.whatRegion(), reg.region_type);
  position.pose.position.x = 14.789078;
  position.pose.position.y = -1.999240;

  observer->rgobs.updatePosition(px, py);
  observer->rgobs.checkWithin();
  observer->rgobs.checkVisited();
  EXPECT_EQ(observer->rgobs.isWithin(), true);
  EXPECT_EQ(observer->rgobs.isVisited(), false);

  position.pose.position.x = 23.777065;
  position.pose.position.y = -2.297022;

  observer->rgobs.updatePosition(px, py);
  observer->rgobs.checkWithin();
  observer->rgobs.checkVisited();
  EXPECT_EQ(observer->rgobs.isWithin(), false);
  EXPECT_EQ(observer->rgobs.isVisited(), true);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "RegionObserverNodeTest");
    return RUN_ALL_TESTS();
}
