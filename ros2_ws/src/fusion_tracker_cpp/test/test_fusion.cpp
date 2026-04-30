#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fusion_tracker_cpp/ekf_tracker.hpp"
#include "fusion_tracker_cpp/hungarian_assigner.hpp"

TEST(FusionCore, HungarianAssigner) {
  fusion_tracker_cpp::HungarianAssigner assigner;
  std::vector<std::vector<double>> cost{{1.0, 4.0}, {3.0, 2.0}};
  auto matches = assigner.assign(cost, 10.0);
  EXPECT_EQ(matches.size(), 2U);
}

TEST(FusionCore, EkfUpdateMovesState) {
  fusion_tracker_cpp::EkfTracker ekf;
  Eigen::Vector3d z(10.0, 0.0, 0.0);
  ekf.updatePosition(z, Eigen::Matrix3d::Identity());
  EXPECT_GT(ekf.state()(0), 0.0);
}
