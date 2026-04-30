#pragma once

#include <Eigen/Dense>

namespace fusion_tracker_cpp {

class EkfTracker {
public:
  EkfTracker();
  void predict(double dt);
  void updatePosition(const Eigen::Vector3d &z, const Eigen::Matrix3d &r);

  const Eigen::VectorXd &state() const { return x_; }
  const Eigen::MatrixXd &covariance() const { return p_; }
  double covarianceTrace() const { return p_.trace(); }

private:
  Eigen::VectorXd x_;
  Eigen::MatrixXd p_;
};

}  // namespace fusion_tracker_cpp
