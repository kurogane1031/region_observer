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
    std::vector<int> region_type {1,2,3}; // outdoor, crosswalk, traffic light
    observer->setRegionOfInterest(region_type, x_l, y_l, x_u, y_u);
  }
  void TearDown() {}
};

TEST_F(RegionObserverTest, whatRegionIsThis){
  std::vector<int> test_what_region {
    static_cast<int>(perception::RegionType::OUTDOOR),
    static_cast<int>(perception::RegionType::CROSSWALK),
    static_cast<int>(perception::RegionType::TRAFFIC_LIGHT)};
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

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
