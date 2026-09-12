#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "opponent_tracker/cluster_extractor.hpp"
#include "opponent_tracker/kalman_tracker_2d.hpp"

namespace opponent_tracker
{

class OpponentTrackerNode : public rclcpp::Node
{
public:
  explicit OpponentTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void initParameters();
  void initStaticObstacles();
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void publishMarkers(const rclcpp::Time & stamp, const Cluster & detected_cluster);

  // Parse raw PointCloud2 message to Point3D vector without PCL
  bool parsePointCloud2(
    const sensor_msgs::msg::PointCloud2 & msg,
    std::vector<Point3D> & out_points,
    const Eigen::Affine3f & transform_to_map) const;

  // ROS 2 Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_bucket_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_velocity_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // TF2 Buffer & Listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Core Processing Components
  ClusterExtractor extractor_;
  KalmanTracker2D tracker_;

  std::string target_frame_{"map"};
  rclcpp::Time last_stamp_;
  bool first_frame_{true};
};

}  // namespace opponent_tracker

