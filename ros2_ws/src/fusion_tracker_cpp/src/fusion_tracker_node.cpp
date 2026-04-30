#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <mosaic_msgs/msg/detection3_d.hpp>
#include <mosaic_msgs/msg/detection3_d_array.hpp>
#include <mosaic_msgs/msg/track.hpp>
#include <mosaic_msgs/msg/track_array.hpp>

#include "fusion_tracker_cpp/ekf_tracker.hpp"
#include "fusion_tracker_cpp/hungarian_assigner.hpp"

namespace {

double axisAlignedIou(const Eigen::Vector3d &c_a, const Eigen::Vector3d &s_a, const Eigen::Vector3d &c_b,
                      const Eigen::Vector3d &s_b) {
  const Eigen::Vector3d min_a = c_a - 0.5 * s_a;
  const Eigen::Vector3d max_a = c_a + 0.5 * s_a;
  const Eigen::Vector3d min_b = c_b - 0.5 * s_b;
  const Eigen::Vector3d max_b = c_b + 0.5 * s_b;

  const Eigen::Vector3d inter_min = min_a.cwiseMax(min_b);
  const Eigen::Vector3d inter_max = max_a.cwiseMin(max_b);
  const Eigen::Vector3d inter_size = (inter_max - inter_min).cwiseMax(Eigen::Vector3d::Zero());
  const double inter_vol = inter_size.x() * inter_size.y() * inter_size.z();
  const double vol_a = std::max(0.0, s_a.x()) * std::max(0.0, s_a.y()) * std::max(0.0, s_a.z());
  const double vol_b = std::max(0.0, s_b.x()) * std::max(0.0, s_b.y()) * std::max(0.0, s_b.z());
  const double denom = vol_a + vol_b - inter_vol;
  if (denom <= 1e-9) {
    return 0.0;
  }
  return inter_vol / denom;
}

Eigen::Matrix3d measurementNoise(const mosaic_msgs::msg::Detection3D &det) {
  const double base = (det.source == "lidar") ? 0.35 : 1.0;
  return Eigen::Matrix3d::Identity() * base;
}

struct TrackState {
  fusion_tracker_cpp::EkfTracker ekf;
  std::string status{"tentative"};
  int hits{0};
  int misses{0};
  Eigen::Vector3d size{4.5, 1.8, 1.6};
};

}  // namespace

class FusionTrackerNode : public rclcpp::Node {
public:
  FusionTrackerNode() : Node("fusion_tracker_node"), next_id_(1) {
    declare_parameter("prediction_dt", 0.1);
    declare_parameter("max_assignment_cost", 12.0);
    declare_parameter("mahalanobis_gate", 9.21);
    declare_parameter("iou_weight", 2.0);
    declare_parameter("confirm_hits", 3);
    declare_parameter("tentative_max_misses", 2);
    declare_parameter("confirmed_max_misses", 8);

    cam_sub_ = create_subscription<mosaic_msgs::msg::Detection3DArray>(
        "/mosaic/detections/camera", 10,
        std::bind(&FusionTrackerNode::cameraCallback, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<mosaic_msgs::msg::Detection3DArray>(
        "/mosaic/detections/lidar", 10,
        std::bind(&FusionTrackerNode::lidarCallback, this, std::placeholders::_1));
    tracks_pub_ = create_publisher<mosaic_msgs::msg::TrackArray>("/mosaic/tracks", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&FusionTrackerNode::tick, this));
    RCLCPP_INFO(get_logger(), "Fusion tracker node started (Hungarian assignment + EKF).");
  }

private:
  void cameraCallback(const mosaic_msgs::msg::Detection3DArray::SharedPtr msg) { latest_cam_ = *msg; }
  void lidarCallback(const mosaic_msgs::msg::Detection3DArray::SharedPtr msg) { latest_lidar_ = *msg; }

  void tick() {
    const double dt = get_parameter("prediction_dt").as_double();
    const double max_cost = get_parameter("max_assignment_cost").as_double();
    const double gate = get_parameter("mahalanobis_gate").as_double();
    const double iou_w = get_parameter("iou_weight").as_double();
    const int confirm_hits = get_parameter("confirm_hits").as_int();
    const int tentative_max_misses = get_parameter("tentative_max_misses").as_int();
    const int confirmed_max_misses = get_parameter("confirmed_max_misses").as_int();

    std::vector<mosaic_msgs::msg::Detection3D> dets = latest_cam_.detections;
    dets.insert(dets.end(), latest_lidar_.detections.begin(), latest_lidar_.detections.end());

    for (auto &entry : tracks_) {
      entry.second.ekf.predict(dt);
      entry.second.misses++;
      if (entry.second.status == "confirmed" && entry.second.misses > 0) {
        entry.second.status = "coasting";
      }
    }

    if (!dets.empty() && !tracks_.empty()) {
      std::vector<int> track_ids;
      track_ids.reserve(tracks_.size());
      for (const auto &kv : tracks_) {
        track_ids.push_back(kv.first);
      }
      std::sort(track_ids.begin(), track_ids.end());

      const std::size_t n = track_ids.size();
      const std::size_t m = dets.size();
      constexpr double k_big = 1e9;
      const std::size_t s_dim = n + m;
      std::vector<std::vector<double>> cost(s_dim, std::vector<double>(s_dim, k_big));

      for (std::size_t c = 0; c < m; ++c) {
        const auto &det = dets[c];
        const Eigen::Vector3d z(det.center.x, det.center.y, det.center.z);
        const Eigen::Vector3d det_size(det.size.x, det.size.y, det.size.z);

        for (std::size_t track_idx = 0; track_idx < n; ++track_idx) {
          const int id = track_ids[track_idx];
          auto &tr = tracks_.at(id);
          const Eigen::VectorXd &x = tr.ekf.state();
          const Eigen::Vector3d x_pos(x(0), x(1), x(2));

          const Eigen::MatrixXd &p = tr.ekf.covariance();
          const Eigen::Matrix3d p_pos = p.block<3, 3>(0, 0);
          const Eigen::Matrix3d meas_r = measurementNoise(det);
          const Eigen::Matrix3d s = p_pos + meas_r;

          const Eigen::Vector3d innov = z - x_pos;
          double mahal = std::numeric_limits<double>::infinity();
          const double det_s = s.determinant();
          if (det_s > 1e-9) {
            mahal = innov.transpose() * s.inverse() * innov;
          }

          if (!(mahal <= gate)) {
            continue;
          }

          const double iou = axisAlignedIou(x_pos, tr.size, z, det_size);
          cost[track_idx][c] = mahal + iou_w * (1.0 - iou);
        }
      }

      for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t k = 0; k < n; ++k) {
          cost[i][m + k] = (i == k) ? 0.0 : k_big;
        }
      }
      for (std::size_t d = 0; d < m; ++d) {
        for (std::size_t j = 0; j < m; ++j) {
          cost[n + d][j] = (d == j) ? 0.0 : k_big;
        }
        for (std::size_t k = 0; k < n; ++k) {
          cost[n + d][m + k] = k_big;
        }
      }

      fusion_tracker_cpp::HungarianAssigner hungarian;
      const std::vector<std::size_t> col_for_row = hungarian.assignSquare(cost);
      std::vector<char> det_used(dets.size(), 0);

      for (std::size_t r = 0; r < n; ++r) {
        const std::size_t c = col_for_row[r];
        if (c >= m) {
          continue;
        }
        if (cost[r][c] > max_cost || cost[r][c] >= k_big * 0.5) {
          continue;
        }
        const int id = track_ids[r];
        auto &tr = tracks_.at(id);
        const auto &det = dets[c];

        const Eigen::Vector3d z(det.center.x, det.center.y, det.center.z);
        tr.ekf.updatePosition(z, measurementNoise(det));
        tr.size = Eigen::Vector3d(det.size.x, det.size.y, det.size.z);
        tr.misses = 0;
        tr.hits++;
        if (tr.hits >= confirm_hits) {
          tr.status = "confirmed";
        } else {
          tr.status = "tentative";
        }
        det_used[c] = 1;
      }

      for (std::size_t c = 0; c < dets.size(); ++c) {
        if (det_used[c]) {
          continue;
        }
        createTrack(dets[c]);
      }
    } else if (!dets.empty() && tracks_.empty()) {
      for (const auto &det : dets) {
        createTrack(det);
      }
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
      auto &tr = it->second;
      const bool tentative = (tr.status == "tentative");
      const int miss_limit = tentative ? tentative_max_misses : confirmed_max_misses;
      if (tr.misses > miss_limit) {
        it = tracks_.erase(it);
      } else {
        ++it;
      }
    }

    publishTracks();
  }

  void createTrack(const mosaic_msgs::msg::Detection3D &det) {
    TrackState state;
    state.size = Eigen::Vector3d(det.size.x, det.size.y, det.size.z);
    const Eigen::Vector3d z(det.center.x, det.center.y, det.center.z);
    state.ekf.updatePosition(z, measurementNoise(det));
    state.hits = 1;
    state.misses = 0;
    state.status = "tentative";
    tracks_[next_id_++] = std::move(state);
  }

  void publishTracks() {
    mosaic_msgs::msg::TrackArray out;
    out.header.stamp = now();
    out.header.frame_id = "base_link";
    for (const auto &entry : tracks_) {
      mosaic_msgs::msg::Track t;
      t.header = out.header;
      t.track_id = entry.first;
      t.status = entry.second.status;
      const auto &x = entry.second.ekf.state();
      t.position.x = x(0);
      t.position.y = x(1);
      t.position.z = x(2);
      t.velocity.x = x(3);
      t.velocity.y = x(4);
      t.velocity.z = x(5);
      t.acceleration.x = x(6);
      t.acceleration.y = x(7);
      t.acceleration.z = x(8);
      t.covariance_trace = static_cast<float>(entry.second.ekf.covarianceTrace());
      out.tracks.push_back(t);
    }
    tracks_pub_->publish(out);
  }

  int next_id_;
  std::unordered_map<int, TrackState> tracks_;
  mosaic_msgs::msg::Detection3DArray latest_cam_;
  mosaic_msgs::msg::Detection3DArray latest_lidar_;

  rclcpp::Subscription<mosaic_msgs::msg::Detection3DArray>::SharedPtr cam_sub_;
  rclcpp::Subscription<mosaic_msgs::msg::Detection3DArray>::SharedPtr lidar_sub_;
  rclcpp::Publisher<mosaic_msgs::msg::TrackArray>::SharedPtr tracks_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
