#include "fusion_tracker_cpp/ekf_tracker.hpp"

namespace fusion_tracker_cpp {

EkfTracker::EkfTracker() : x_(Eigen::VectorXd::Zero(9)), p_(Eigen::MatrixXd::Identity(9, 9)) {}

void EkfTracker::predict(double dt) {
  Eigen::MatrixXd f = Eigen::MatrixXd::Identity(9, 9);
  for (int i = 0; i < 3; ++i) {
    f(i, i + 3) = dt;
    f(i, i + 6) = 0.5 * dt * dt;
    f(i + 3, i + 6) = dt;
  }
  x_ = f * x_;
  p_ = f * p_ * f.transpose() + 0.05 * Eigen::MatrixXd::Identity(9, 9);
}

void EkfTracker::updatePosition(const Eigen::Vector3d &z, const Eigen::Matrix3d &r) {
  Eigen::Matrix<double, 3, 9> h = Eigen::Matrix<double, 3, 9>::Zero();
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  h(2, 2) = 1.0;

  const Eigen::Vector3d y = z - h * x_;
  const Eigen::Matrix3d s = h * p_ * h.transpose() + r;
  const Eigen::Matrix<double, 9, 3> k = p_ * h.transpose() * s.inverse();
  x_ = x_ + k * y;
  p_ = (Eigen::MatrixXd::Identity(9, 9) - k * h) * p_;
}

}  // namespace fusion_tracker_cpp
