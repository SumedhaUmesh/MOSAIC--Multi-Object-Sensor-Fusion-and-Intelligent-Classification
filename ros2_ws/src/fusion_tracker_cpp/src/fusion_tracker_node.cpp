#include <algorithm>
#include <limits>
#include <memory>
#include <tuple>
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

std::vector<std::pair<std::size_t, std::size_t>> greedyMinCostAssignment(const std::vector<std::tuple<double, std::size_t, std::size_t>> &candidates,
                                                                          std::size_t row_count, std::size_t col_count, double max_cost) {
  std::vector<std::tuple<double, std::size_t, std::size_t>> sorted = candidates;
  std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) { return std::get<0>(a) < std::get<0>(b); });

  std::vector<char> used_row(row_count, 0);
  std::vector<char> used_col(col_count, 0);
  std::vector<std::pair<std::size_t, std::size_t>> matches;

  for (const auto &entry : sorted) {
    const double cost = std::get<0>(entry);
    const std::size_t r = std::get<1>(entry);
    const std::size_t c = std::get<2>(entry);
    if (cost > max_cost) {
      continue;
    }
    if (r >= used_row.size() || c >= used_col.size()) {
      continue;
    }
    if (used_row[r] || used_col[c]) {
      continue;
    }
    used_row[r] = 1;
    used_col[c] = 1;
    matches.emplace_back(r, c);
  }
  return matches;
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
    RCLCPP_INFO(get_logger(), "Fusion tracker node started (gated assignment + EKF).");
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

      std::vector<std::tuple<double, std::size_t, std::size_t>> candidates;
      candidates.reserve(track_ids.size() * dets.size());

      for (std::size_t c = 0; c < dets.size(); ++c) {
        const auto &det = dets[c];
        const Eigen::Vector3d z(det.center.x, det.center.y, det.center.z);
        const Eigen::Vector3d det_size(det.size.x, det.size.y, det.size.z);

        for (std::size_t track_idx = 0; track_idx < track_ids.size(); ++track_idx) {
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
          const double cost = mahal + iou_w * (1.0 - iou);
          candidates.emplace_back(cost, track_idx, c);
        }
      }

      const auto matches = greedyMinCostAssignment(candidates, track_ids.size(), dets.size(), max_cost);
      std::vector<char> det_used(dets.size(), 0);

      for (const auto &m : matches) {
        const std::size_t r = m.first;
        const std::size_t c = m.second;
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
