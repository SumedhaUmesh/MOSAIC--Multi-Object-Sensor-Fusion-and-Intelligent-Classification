#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mosaic_msgs/msg/detection3_d_array.hpp>
#include <mosaic_msgs/msg/detection3_d.hpp>

class LidarPerceptionNode : public rclcpp::Node {
public:
  LidarPerceptionNode() : Node("lidar_perception_node") {
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mosaic/lidar/points", 10,
        std::bind(&LidarPerceptionNode::pointCloudCallback, this, std::placeholders::_1));
    pub_ = create_publisher<mosaic_msgs::msg::Detection3DArray>("/mosaic/detections/lidar", 10);
    RCLCPP_INFO(get_logger(), "LiDAR perception node started (clustering stub mode).");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    mosaic_msgs::msg::Detection3DArray out;
    out.header = msg->header;

    mosaic_msgs::msg::Detection3D det;
    det.header = msg->header;
    det.id = 1;
    det.source = "lidar";
    det.class_label = "car";
    det.center.x = 9.5;
    det.center.y = 0.2;
    det.center.z = 0.0;
    det.size.x = 4.2;
    det.size.y = 2.0;
    det.size.z = 1.7;
    det.confidence = 0.85F;
    out.detections.push_back(det);

    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<mosaic_msgs::msg::Detection3DArray>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
