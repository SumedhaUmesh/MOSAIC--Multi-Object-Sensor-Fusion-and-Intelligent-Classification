#include <unordered_map>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mosaic_msgs/msg/detection3_d_array.hpp>
#include <mosaic_msgs/msg/track_array.hpp>
#include <mosaic_msgs/msg/track.hpp>
#include <Eigen/Dense>

#include "fusion_tracker_cpp/ekf_tracker.hpp"
#include "fusion_tracker_cpp/hungarian_assigner.hpp"

struct TrackState {
  fusion_tracker_cpp::EkfTracker ekf;
  std::string status{"tentative"};
  int misses{0};
};

class FusionTrackerNode : public rclcpp::Node {
public:
  FusionTrackerNode() : Node("fusion_tracker_node"), next_id_(1) {
    cam_sub_ = create_subscription<mosaic_msgs::msg::Detection3DArray>(
        "/mosaic/detections/camera", 10,
        std::bind(&FusionTrackerNode::cameraCallback, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<mosaic_msgs::msg::Detection3DArray>(
        "/mosaic/detections/lidar", 10,
        std::bind(&FusionTrackerNode::lidarCallback, this, std::placeholders::_1));
    tracks_pub_ = create_publisher<mosaic_msgs::msg::TrackArray>("/mosaic/tracks", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&FusionTrackerNode::tick, this));
    RCLCPP_INFO(get_logger(), "Fusion tracker node started.");
  }

private:
  void cameraCallback(const mosaic_msgs::msg::Detection3DArray::SharedPtr msg) { latest_cam_ = *msg; }
  void lidarCallback(const mosaic_msgs::msg::Detection3DArray::SharedPtr msg) { latest_lidar_ = *msg; }

  void tick() {
    std::vector<mosaic_msgs::msg::Detection3D> merged = latest_cam_.detections;
    merged.insert(merged.end(), latest_lidar_.detections.begin(), latest_lidar_.detections.end());
    for (auto &entry : tracks_) {
      entry.second.ekf.predict(0.1);
      entry.second.misses++;
      if (entry.second.misses > 5) {
        entry.second.status = "coasting";
      }
    }

    for (const auto &det : merged) {
      if (tracks_.empty()) {
        createTrack(det);
        continue;
      }
      // Simplified nearest-track update.
      auto it = tracks_.begin();
      Eigen::Vector3d z(det.center.x, det.center.y, det.center.z);
      Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * ((det.source == "lidar") ? 0.4 : 1.0);
      it->second.ekf.updatePosition(z, r);
      it->second.misses = 0;
      it->second.status = "confirmed";
    }
    publishTracks();
  }

  void createTrack(const mosaic_msgs::msg::Detection3D &det) {
    TrackState state;
    Eigen::Vector3d z(det.center.x, det.center.y, det.center.z);
    state.ekf.updatePosition(z, Eigen::Matrix3d::Identity());
    tracks_[next_id_++] = state;
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
