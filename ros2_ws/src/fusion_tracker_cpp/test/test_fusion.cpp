#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fusion_tracker_cpp/ekf_tracker.hpp"
#include "fusion_tracker_cpp/hungarian_assigner.hpp"

TEST(FusionCore, HungarianAssignerOptimal2x2) {
  fusion_tracker_cpp::HungarianAssigner assigner;
  const std::vector<std::vector<double>> cost{{1.0, 4.0}, {3.0, 2.0}};
  const auto col = assigner.assignSquare(cost);
  ASSERT_EQ(col.size(), 2U);
  EXPECT_EQ(col[0], 0U);
  EXPECT_EQ(col[1], 1U);
}

TEST(FusionCore, HungarianAssignerOptimal3x3) {
  // Minimum assignment cost is 10 (e.g. columns 2,1,0); a greedy pass over sorted edges yields 14.
  fusion_tracker_cpp::HungarianAssigner assigner;
  const std::vector<std::vector<double>> cost{{1.0, 2.0, 3.0}, {2.0, 4.0, 6.0}, {3.0, 6.0, 9.0}};
  const auto col = assigner.assignSquare(cost);
  ASSERT_EQ(col.size(), 3U);
  double sum = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    sum += cost[i][col[i]];
  }
  EXPECT_NEAR(sum, 10.0, 1e-9);
}

TEST(FusionCore, EkfUpdateMovesState) {
  fusion_tracker_cpp::EkfTracker ekf;
  Eigen::Vector3d z(10.0, 0.0, 0.0);
  ekf.updatePosition(z, Eigen::Matrix3d::Identity());
  EXPECT_GT(ekf.state()(0), 0.0);
}
