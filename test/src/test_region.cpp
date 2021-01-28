#include "region_observer/region_observer.h"
#include <gtest/gtest.h>
#include <string>

struct RegionObserverTest : public testing::Test {
  std::unique_ptr<perception::RegionObserver> observer {nullptr};
  void SetUp() {
    observer = std::make_unique<perception::RegionObserver>();
  }
  void TearDown() {}
};

TEST_F(RegionObserverTest, visitedOnce){
  double x_l {-0.9};
  double y_l {-0.9};
  double x_u {-0.8};
  double y_u {-0.8};
  observer->setRegionOfInterest(x_l, y_l, x_u, y_u);

  double px = -0.75;
  double py = -0.75;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  ASSERT_EQ(observer->isVisited(), false);

  px = -0.81;
  py = -0.81;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), true);
  ASSERT_EQ(observer->isVisited(), false);

  px = -0.91;
  py = -0.91;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  EXPECT_EQ(observer->isVisited(), true);
}

TEST_F(RegionObserverTest, visitedTwice){
  double x_l {-0.9};
  double y_l {-0.9};
  double x_u {-0.8};
  double y_u {-0.8};
  observer->setRegionOfInterest(x_l, y_l, x_u, y_u);

  double px = -0.75;
  double py = -0.75;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  ASSERT_EQ(observer->isVisited(), false);

  px = -0.81;
  py = -0.81;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), true);
  ASSERT_EQ(observer->isVisited(), false);

  px = -0.91;
  py = -0.91;
  observer->updatePosition(px, py);
  observer->checkWithin();
  observer->checkVisited();
  ASSERT_EQ(observer->isWithin(), false);
  EXPECT_EQ(observer->isVisited(), true);

  px = -0.81;
  py = -0.81;
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
