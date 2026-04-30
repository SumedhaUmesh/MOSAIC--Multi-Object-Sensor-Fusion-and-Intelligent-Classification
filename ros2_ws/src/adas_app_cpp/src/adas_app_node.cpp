#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mosaic_msgs/msg/track_array.hpp>
#include <mosaic_msgs/msg/adas_warning.hpp>

class AdasAppNode : public rclcpp::Node {
public:
  AdasAppNode() : Node("adas_app_node") {
    sub_ = create_subscription<mosaic_msgs::msg::TrackArray>(
        "/mosaic/tracks", 10, std::bind(&AdasAppNode::tracksCallback, this, std::placeholders::_1));
    pub_ = create_publisher<mosaic_msgs::msg::AdasWarning>("/mosaic/adas/warnings", 10);
    declare_parameter("ttc_threshold", 2.5);
    RCLCPP_INFO(get_logger(), "ADAS app node started.");
  }

private:
  void tracksCallback(const mosaic_msgs::msg::TrackArray::SharedPtr msg) {
    const double ttc_threshold = get_parameter("ttc_threshold").as_double();
    for (const auto &track : msg->tracks) {
      const double rel_x = track.position.x;
      const double rel_vx = track.velocity.x;
      if (rel_vx < -0.1) {
        const double ttc = std::abs(rel_x / rel_vx);
        if (ttc < ttc_threshold) {
          mosaic_msgs::msg::AdasWarning w;
          w.header = msg->header;
          w.warning_type = "forward_collision";
          w.track_id = track.track_id;
          w.severity = static_cast<float>(1.0 / std::max(ttc, 0.1));
          w.message = "Potential forward collision detected";
          pub_->publish(w);
        }
      }
    }
  }

  rclcpp::Subscription<mosaic_msgs::msg::TrackArray>::SharedPtr sub_;
  rclcpp::Publisher<mosaic_msgs::msg::AdasWarning>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdasAppNode>());
  rclcpp::shutdown();
  return 0;
}
