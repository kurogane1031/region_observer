#include "region_observer/region_observer.h"
#include <gtest/gtest.h>

TEST(JustATest, returnTrue){
  bool val1{true};
  bool val2{true};
  EXPECT_EQ(val1, val2);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
