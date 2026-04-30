#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <mosaic_msgs/msg/adas_warning.hpp>
#include <mosaic_msgs/msg/track_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

bool parseLaneDepartureJson(const std::string &json, bool *departure, std::optional<double> *offset_px) {
  if (!departure || !offset_px) {
    return false;
  }

  const auto pos = json.find("\"lane_departure\"");
  if (pos == std::string::npos) {
    return false;
  }

  const auto true_token = json.find("true", pos);
  const auto false_token = json.find("false", pos);
  if (true_token == std::string::npos && false_token == std::string::npos) {
    return false;
  }

  if (true_token != std::string::npos && (false_token == std::string::npos || true_token < false_token)) {
    *departure = true;
  } else {
    *departure = false;
  }

  offset_px->reset();
  const auto key = std::string("\"offset_from_image_center_px\"");
  const auto kpos = json.find(key);
  if (kpos != std::string::npos) {
    const auto colon = json.find(':', kpos + key.size());
    if (colon != std::string::npos) {
      try {
        const auto substr = json.substr(colon + 1);
        *offset_px = std::stod(substr);
      } catch (...) {
        offset_px->reset();
      }
    }
  }

  return true;
}

}  // namespace

class AdasAppNode : public rclcpp::Node {
public:
  AdasAppNode() : Node("adas_app_node"), last_lane_departure_(false) {
    tracks_sub_ = create_subscription<mosaic_msgs::msg::TrackArray>(
        "/mosaic/tracks", 10, std::bind(&AdasAppNode::tracksCallback, this, std::placeholders::_1));
    lanes_sub_ = create_subscription<std_msgs::msg::String>(
        "/mosaic/lanes/state", 10, std::bind(&AdasAppNode::lanesCallback, this, std::placeholders::_1));
    pub_ = create_publisher<mosaic_msgs::msg::AdasWarning>("/mosaic/adas/warnings", 10);
    declare_parameter("ttc_threshold", 2.5);
    declare_parameter("publish_fcw_on_rising_edge_only", true);
    declare_parameter("publish_ldw_on_rising_edge_only", true);
    RCLCPP_INFO(get_logger(), "ADAS app node started.");
  }

private:
  void tracksCallback(const mosaic_msgs::msg::TrackArray::SharedPtr msg) {
    const double ttc_threshold = get_parameter("ttc_threshold").as_double();
    const bool fcw_rising_only = get_parameter("publish_fcw_on_rising_edge_only").as_bool();
    for (const auto &track : msg->tracks) {
      const double rel_x = track.position.x;
      const double rel_vx = track.velocity.x;
      const int tid = track.track_id;
      double ttc = std::numeric_limits<double>::infinity();
      bool in_zone = false;
      if (rel_vx < -0.1) {
        ttc = std::abs(rel_x / rel_vx);
        in_zone = (ttc < ttc_threshold);
      }
      const bool was_in_zone = fcw_zone_latched_[tid];
      if (in_zone) {
        if (!fcw_rising_only || !was_in_zone) {
          mosaic_msgs::msg::AdasWarning w;
          w.header = msg->header;
          w.warning_type = "forward_collision";
          w.track_id = tid;
          w.severity = static_cast<float>(1.0 / std::max(ttc, 0.1));
          w.message = "Potential forward collision detected";
          pub_->publish(w);
        }
        fcw_zone_latched_[tid] = true;
      } else {
        fcw_zone_latched_[tid] = false;
      }
    }
  }

  void lanesCallback(const std_msgs::msg::String::SharedPtr msg) {
    bool departure = false;
    std::optional<double> offset_px;
    if (!parseLaneDepartureJson(msg->data, &departure, &offset_px)) {
      return;
    }

    const bool rising_only = get_parameter("publish_ldw_on_rising_edge_only").as_bool();
    const bool rising = departure && !last_lane_departure_;
    last_lane_departure_ = departure;

    if (!departure) {
      return;
    }
    if (rising_only && !rising) {
      return;
    }

    mosaic_msgs::msg::AdasWarning w;
    w.header.stamp = now();
    w.header.frame_id = "base_link";
    w.warning_type = "lane_departure";
    w.track_id = 0;
    w.severity = 1.0F;
    if (offset_px.has_value()) {
      w.severity = static_cast<float>(std::min(5.0, std::abs(offset_px.value()) / 20.0));
      w.message = "Lane departure detected (lateral offset from lane center)";
    } else {
      w.message = "Lane departure detected";
    }
    pub_->publish(w);
  }

  bool last_lane_departure_;
  std::unordered_map<int, bool> fcw_zone_latched_;

  rclcpp::Subscription<mosaic_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lanes_sub_;
  rclcpp::Publisher<mosaic_msgs::msg::AdasWarning>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdasAppNode>());
  rclcpp::shutdown();
  return 0;
}
