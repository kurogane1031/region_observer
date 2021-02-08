#include "region_observer/region_observer.h"
#include <gtest/gtest.h>
#include <string>

struct RegionObserverTest : public testing::Test {
  std::unique_ptr<perception::RegionObserver> observer {nullptr};
  void SetUp() {
    observer = std::make_unique<perception::RegionObserver>();
    double x_l {-0.9};
    double y_l {-0.9};
    double x_u {-0.8};
    double y_u {-0.8};
    std::vector<unsigned int> region_type {0,1,1,1,0,0}; // outdoor, crosswalk, traffic light
    observer->setRegionOfInterest(region_type, x_l, y_l, x_u, y_u);
  }
  void TearDown() {}
};

TEST_F(RegionObserverTest, whatRegionIsThis){
  std::vector<unsigned int> test_what_region (static_cast<size_t>(perception::RegionType::RegionType_size_t), 0);
  test_what_region[static_cast<unsigned int>(perception::RegionType::OUTDOOR)] = 1;
  test_what_region[static_cast<unsigned int>(perception::RegionType::CROSSWALK)] = 1;
  test_what_region[static_cast<unsigned int>(perception::RegionType::TRAFFIC_LIGHT)] = 1;
  EXPECT_EQ(observer->whatRegion(),test_what_region);
}

TEST_F(RegionObserverTest, enteringFromSide){
  double px = -0.8;
  double py = -0.75;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), false);

  // going left
  px = -0.8;
  py = -0.79;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), false);

  // going left
  px = -0.8;
  py = -0.81;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), true);
}

TEST_F(RegionObserverTest, enteringFromTop){
  double px = -1.0;
  double py = -1.0;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), false);

  // going up
  px = -0.95;
  py = -0.95;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), false);

  // going up
  px = -0.85;
  py = -0.85;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), true);

  // going up
  px = -0.75;
  py = -0.75;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), false);
}

TEST_F(RegionObserverTest, visitedOnce){
  double px = -0.75;
  double py = -0.75;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  ASSERT_EQ(observer->isVisited(), false);

  // going down
  px = -0.81;
  py = -0.81;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), true);
  ASSERT_EQ(observer->isVisited(), false);

  // going down
  px = -0.91;
  py = -0.91;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  EXPECT_EQ(observer->isVisited(), true);
}

TEST_F(RegionObserverTest, visitedTwice){
  double px = -0.75;
  double py = -0.75;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  ASSERT_EQ(observer->isVisited(), false);

  // going down
  px = -0.81;
  py = -0.81;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), true);
  ASSERT_EQ(observer->isVisited(), false);

  // going down
  px = -0.91;
  py = -0.91;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  EXPECT_EQ(observer->isVisited(), true);

  // going up
  px = -0.81;
  py = -0.81;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), false);
  EXPECT_EQ(observer->isVisited(), true);
}

TEST_F(RegionObserverTest, initialStartFromWithin){
  double px = -0.85;
  double py = -0.85;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), true);
  ASSERT_EQ(observer->isVisited(), false);

  // Going up
  px = -0.75;
  py = -0.75;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), false);
  EXPECT_EQ(observer->isVisited(), true);
}

TEST_F(RegionObserverTest, visitTwoDifferentRegion){
  double px = -0.75;
  double py = -0.75;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  ASSERT_EQ(observer->isVisited(), false);

  // going down
  px = -0.81;
  py = -0.81;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), true);
  ASSERT_EQ(observer->isVisited(), false);

  // going down
  px = -0.91;
  py = -0.91;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  EXPECT_EQ(observer->isVisited(), true);

  double x_l {-1.2};
  double y_l {-1.2};
  double x_u {-1.0};
  double y_u {-1.0};
  std::vector<unsigned int> region_type {0,1,1,1,0,0}; // outdoor, crosswalk, traffic light
  observer->setRegionOfInterest(region_type, x_l, y_l, x_u, y_u);
  observer->resetFlags(); // reset is_within and is_visited
  // assume updatePosition always running
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  EXPECT_EQ(observer->isWithin(), false);
  EXPECT_EQ(observer->isVisited(), false);

  // going down
  px = -1.1;
  py = -1.1;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), true);
  ASSERT_EQ(observer->isVisited(), false);

  // going down
  px = -1.3;
  py = -1.3;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  EXPECT_EQ(observer->isVisited(), true);
}
// Obtain subscribed topic
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
