#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <mosaic_msgs/msg/detection3_d.hpp>
#include <mosaic_msgs/msg/detection3_d_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarPerceptionNode : public rclcpp::Node {
public:
  LidarPerceptionNode() : Node("lidar_perception_node") {
    declare_parameter("voxel_leaf_size", 0.25);
    declare_parameter("cluster_tolerance", 0.75);
    declare_parameter("cluster_min_size", 18);
    declare_parameter("cluster_max_size", 25000);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mosaic/lidar/points", 10,
        std::bind(&LidarPerceptionNode::pointCloudCallback, this, std::placeholders::_1));
    pub_ = create_publisher<mosaic_msgs::msg::Detection3DArray>("/mosaic/detections/lidar", 10);
    RCLCPP_INFO(get_logger(), "LiDAR perception node started (PCL clustering mode).");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    using PointT = pcl::PointXYZI;
    using CloudT = pcl::PointCloud<PointT>;

    mosaic_msgs::msg::Detection3DArray out;
    out.header = msg->header;
    CloudT::Ptr raw_cloud(new CloudT());
    pcl::fromROSMsg(*msg, *raw_cloud);
    if (raw_cloud->empty()) {
      pub_->publish(out);
      return;
    }

    // 1) Downsample
    const double leaf_size = get_parameter("voxel_leaf_size").as_double();
    CloudT::Ptr filtered_cloud(new CloudT());
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(raw_cloud);
    voxel.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size),
                      static_cast<float>(leaf_size));
    voxel.filter(*filtered_cloud);
    if (filtered_cloud->empty()) {
      pub_->publish(out);
      return;
    }

    // 2) Ground plane removal via RANSAC
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2);
    seg.setInputCloud(filtered_cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.segment(*inliers, *coefficients);

    CloudT::Ptr nonground_cloud(new CloudT());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*nonground_cloud);
    if (nonground_cloud->empty()) {
      pub_->publish(out);
      return;
    }

    // 3) Euclidean clustering
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(nonground_cloud);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(get_parameter("cluster_tolerance").as_double());
    ec.setMinClusterSize(get_parameter("cluster_min_size").as_int());
    ec.setMaxClusterSize(get_parameter("cluster_max_size").as_int());
    ec.setSearchMethod(tree);
    ec.setInputCloud(nonground_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    std::uint32_t det_id = 0;
    for (const auto &cluster : cluster_indices) {
      float min_x = std::numeric_limits<float>::max();
      float min_y = std::numeric_limits<float>::max();
      float min_z = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float max_y = std::numeric_limits<float>::lowest();
      float max_z = std::numeric_limits<float>::lowest();

      for (int idx : cluster.indices) {
        const auto &p = nonground_cloud->points[static_cast<std::size_t>(idx)];
        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
        min_z = std::min(min_z, p.z);
        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
        max_z = std::max(max_z, p.z);
      }

      const float sx = max_x - min_x;
      const float sy = max_y - min_y;
      const float sz = max_z - min_z;
      if (sx < 1.0F || sy < 0.5F || sz < 0.5F) {
        continue;
      }

      mosaic_msgs::msg::Detection3D det;
      det.header = msg->header;
      det.id = det_id++;
      det.source = "lidar";
      det.class_label = "vehicle";
      det.center.x = 0.5F * (min_x + max_x);
      det.center.y = 0.5F * (min_y + max_y);
      det.center.z = 0.5F * (min_z + max_z);
      det.size.x = sx;
      det.size.y = sy;
      det.size.z = sz;
      det.confidence = 0.9F;
      out.detections.push_back(det);
    }

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
