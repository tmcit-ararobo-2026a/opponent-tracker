#include "opponent_tracker/opponent_tracker_node.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace opponent_tracker
{

OpponentTrackerNode::OpponentTrackerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("opponent_tracker_node", options)
{
  initParameters();
  initStaticObstacles();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const std::string input_topic = this->declare_parameter<std::string>("input_topic", "/dynamic_points");
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::QoS(10),
    std::bind(&OpponentTrackerNode::pointCloudCallback, this, std::placeholders::_1));

  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/opponent_robot/pose", 10);
  pub_bucket_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/opponent_robot/bucket_target", 10);
  pub_velocity_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/opponent_robot/velocity", 10);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/opponent_robot/markers", 10);
  pub_our_robot_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/our_robot/markers", 10);

  RCLCPP_INFO(this->get_logger(), "OpponentTrackerNode initialized. Listening to: %s", input_topic.c_str());
}

void OpponentTrackerNode::initParameters()
{
  target_frame_ = this->declare_parameter<std::string>("target_frame", "map");

  ClusterParams c_params;
  c_params.voxel_size = this->declare_parameter<float>("clustering.voxel_size", 0.08f);
  c_params.cluster_tolerance = this->declare_parameter<float>("clustering.cluster_tolerance", 0.35f);
  c_params.min_cluster_size = this->declare_parameter<int>("clustering.min_cluster_size", 15);
  c_params.max_cluster_size = this->declare_parameter<int>("clustering.max_cluster_size", 10000);

  c_params.robot_min_width = this->declare_parameter<float>("robot.min_width", 0.20f);
  c_params.robot_max_width = this->declare_parameter<float>("robot.max_width", 1.30f);
  c_params.robot_min_depth = this->declare_parameter<float>("robot.min_depth", 0.20f);
  c_params.robot_max_depth = this->declare_parameter<float>("robot.max_depth", 1.30f);
  c_params.robot_min_height = this->declare_parameter<float>("robot.min_height", 0.30f);
  c_params.robot_max_height = this->declare_parameter<float>("robot.max_height", 2.20f);
  c_params.robot_max_ground_z = this->declare_parameter<float>("robot.max_ground_z", 0.30f);

  c_params.bucket_min_z = this->declare_parameter<float>("bucket.min_z", 1.15f);
  c_params.bucket_max_z = this->declare_parameter<float>("bucket.max_z", 2.15f);

  c_params.field_min_x = this->declare_parameter<float>("field.min_x", -6.0f);
  c_params.field_max_x = this->declare_parameter<float>("field.max_x", 6.0f);
  c_params.field_min_y = this->declare_parameter<float>("field.min_y", -6.5f);
  c_params.field_max_y = this->declare_parameter<float>("field.max_y", 6.5f);

  extractor_.setParams(c_params);

  TrackerParams t_params;
  t_params.process_noise_pos = this->declare_parameter<float>("kalman.process_noise_pos", 2.0f);
  t_params.process_noise_vel = this->declare_parameter<float>("kalman.process_noise_vel", 5.0f);
  t_params.process_noise_z = this->declare_parameter<float>("kalman.process_noise_z", 0.20f);
  t_params.measurement_noise_pos = this->declare_parameter<float>("kalman.measurement_noise_pos", 0.02f);
  t_params.measurement_noise_z = this->declare_parameter<float>("kalman.measurement_noise_z", 0.02f);
  t_params.max_association_dist = this->declare_parameter<float>("kalman.max_association_dist", 2.0f);
  t_params.max_missed_frames = this->declare_parameter<int>("kalman.max_missed_frames", 10);
  t_params.min_hits_to_confirm = this->declare_parameter<int>("kalman.min_hits_to_confirm", 2);

  tracker_.setParams(t_params);
}

void OpponentTrackerNode::initStaticObstacles()
{
  // Field CAD obstacles centered at (0,0)
  std::vector<StaticObstacle> static_obs = {
    {"CenterBarrier", Eigen::Vector2f(0.0f, 0.0f),    Eigen::Vector2f(10.5f, 0.6f), 0.0f, 0.25f},
    {"Post_NW",       Eigen::Vector2f(-1.27f, 1.48f),  Eigen::Vector2f(0.35f, 0.35f), 0.0f, 0.65f},
    {"Post_SW",       Eigen::Vector2f(-1.27f, -1.48f), Eigen::Vector2f(0.35f, 0.35f), 0.0f, 0.65f},
    {"Post_NE",       Eigen::Vector2f(1.27f, 1.48f),   Eigen::Vector2f(0.35f, 0.35f), 0.0f, 0.65f},
    {"Post_SE",       Eigen::Vector2f(1.27f, -1.48f),  Eigen::Vector2f(0.35f, 0.35f), 0.0f, 0.65f}
  };

  extractor_.setStaticObstacles(static_obs);
  tracker_.setStaticObstacles(static_obs);
}

bool OpponentTrackerNode::parsePointCloud2(
  const sensor_msgs::msg::PointCloud2 & msg,
  std::vector<Point3D> & out_points,
  const Eigen::Affine3f & transform_to_map) const
{
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int intensity_offset = -1;

  for (const auto & field : msg.fields)
  {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
    else if (field.name == "intensity") intensity_offset = field.offset;
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0)
  {
    return false;
  }

  const size_t total_points = msg.width * msg.height;
  out_points.reserve(total_points);

  const uint8_t * data_ptr = msg.data.data();
  const uint32_t point_step = msg.point_step;

  for (size_t i = 0; i < total_points; ++i)
  {
    const uint8_t * pt_data = data_ptr + i * point_step;
    float rx, ry, rz;
    std::memcpy(&rx, pt_data + x_offset, sizeof(float));
    std::memcpy(&ry, pt_data + y_offset, sizeof(float));
    std::memcpy(&rz, pt_data + z_offset, sizeof(float));

    if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz))
    {
      continue;
    }

    Eigen::Vector3f pt_orig(rx, ry, rz);
    Eigen::Vector3f pt_map = transform_to_map * pt_orig;

    Point3D p;
    p.x = pt_map.x();
    p.y = pt_map.y();
    p.z = pt_map.z();

    if (intensity_offset >= 0)
    {
      std::memcpy(&p.intensity, pt_data + intensity_offset, sizeof(float));
    }

    out_points.push_back(p);
  }

  return true;
}

void OpponentTrackerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  const rclcpp::Time current_stamp(msg->header.stamp);

  // Compute dt for Kalman prediction
  double dt = 0.05;
  if (!first_frame_)
  {
    dt = (current_stamp - last_stamp_).seconds();
    if (dt <= 0.0 || dt > 0.5)
    {
      dt = 0.05;
    }
  }
  first_frame_ = false;
  last_stamp_ = current_stamp;

  // 1. Resolve TF to target frame (map)
  Eigen::Affine3f transform_to_map = Eigen::Affine3f::Identity();
  if (msg->header.frame_id != target_frame_)
  {
    try
    {
      const auto tf_stamped = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.05));
      const Eigen::Isometry3d iso = tf2::transformToEigen(tf_stamped.transform);
      transform_to_map = iso.cast<float>();
    }
    catch (const tf2::TransformException & ex)
    {
      try
      {
        const auto tf_stamped = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id, tf2::TimePointZero);
        const Eigen::Isometry3d iso = tf2::transformToEigen(tf_stamped.transform);
        transform_to_map = iso.cast<float>();
      }
      catch (const tf2::TransformException & ex2)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF lookup error: %s", ex2.what());
        return;
      }
    }
  }

  // 2. Parse points
  std::vector<Point3D> points_in_map;
  if (!parsePointCloud2(*msg, points_in_map, transform_to_map))
  {
    RCLCPP_WARN(this->get_logger(), "Failed to parse point cloud!");
    return;
  }

  // 3. Extract candidate clusters
  std::vector<Cluster> raw_clusters = extractor_.extractClusters(points_in_map);
  std::vector<Cluster> clusters;

  // Filter out clusters too close to our own robot (< 0.9m) or in our own base corner
  for (const auto & c : raw_clusters)
  {
    // Our robot is located at origin of base_link -> in map frame it is transform_to_map * (0,0,0)
    const Eigen::Vector3f our_pos_map = transform_to_map.translation();
    const float dist_to_us = (c.centroid.head<2>() - our_pos_map.head<2>()).norm();
    if (dist_to_us < 0.90f)
    {
      continue; // Filter self robot reflections
    }
    // Robocon 2026: Opponent robot does not spawn inside our blue starting base (X < -2.5m, Y < -2.5m)
    if (!tracker_.isTracking() && c.centroid.x() < -2.5f && c.centroid.y() < -2.5f)
    {
      continue;
    }
    clusters.push_back(c);
  }

  // 4. Associate best cluster to Kalman Tracker
  tracker_.predict(dt);

  bool matched = false;
  Cluster best_cluster;

  if (!clusters.empty())
  {
    if (tracker_.isTracking())
    {
      // Find cluster closest to current predicted position
      float min_dist = std::numeric_limits<float>::max();
      int best_idx = -1;

      for (size_t i = 0; i < clusters.size(); ++i)
      {
        const float d = tracker_.computeDistance(clusters[i].centroid);
        if (d < min_dist && d <= tracker_.getParams().max_association_dist)
        {
          min_dist = d;
          best_idx = static_cast<int>(i);
        }
      }

      if (best_idx >= 0)
      {
        best_cluster = clusters[best_idx];
        matched = true;
      }
    }
    else
    {
      // Pick cluster with best robot geometry and bucket confidence in opponent area
      best_cluster = clusters[0];
      matched = true;
    }
  }

  if (matched)
  {
    tracker_.update(best_cluster.centroid, best_cluster.bucket_center, best_cluster.bbox.dimensions);
  }
  else
  {
    tracker_.markMissed();
  }

  // 5. Publish tracking results if confirmed tracking
  if (tracker_.isTracking())
  {
    const Eigen::Vector2f pos = tracker_.getPosition();
    const Eigen::Vector2f vel = tracker_.getVelocity();
    const Eigen::Vector3f bucket = tracker_.getBucketTarget();
    const float heading = tracker_.getHeading();

    // /opponent_robot/pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = current_stamp;
    pose_msg.header.frame_id = target_frame_;
    pose_msg.pose.position.x = pos.x();
    pose_msg.pose.position.y = pos.y();
    pose_msg.pose.position.z = 0.0;

    Eigen::Quaternionf q(Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitZ()));
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pub_pose_->publish(pose_msg);

    // /opponent_robot/bucket_target (Aiming point for cloth shooter)
    geometry_msgs::msg::PointStamped bucket_msg;
    bucket_msg.header.stamp = current_stamp;
    bucket_msg.header.frame_id = target_frame_;
    bucket_msg.point.x = bucket.x();
    bucket_msg.point.y = bucket.y();
    bucket_msg.point.z = bucket.z();
    pub_bucket_->publish(bucket_msg);

    // /opponent_robot/velocity
    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header.stamp = current_stamp;
    vel_msg.header.frame_id = target_frame_;
    vel_msg.twist.linear.x = vel.x();
    vel_msg.twist.linear.y = vel.y();
    vel_msg.twist.linear.z = 0.0;
    pub_velocity_->publish(vel_msg);

    // /opponent_robot/markers
    publishMarkers(current_stamp, matched ? best_cluster : Cluster{});
  }

  // Always publish our own robot marker
  publishOurRobotMarker(current_stamp);
}

void OpponentTrackerNode::publishMarkers(const rclcpp::Time & stamp, const Cluster & detected_cluster)
{
  (void)detected_cluster;
  visualization_msgs::msg::MarkerArray markers;

  const Eigen::Vector2f pos = tracker_.getPosition();
  const Eigen::Vector2f vel = tracker_.getVelocity();
  const Eigen::Vector3f bucket = tracker_.getBucketTarget();
  const float speed = tracker_.getSpeed();
  const float heading = tracker_.getHeading();

  Eigen::Quaternionf q_heading(Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitZ()));

  // 1. Robot 3D Bounding Box (Rock-solid clean chassis box, no jitter or vertical stretching)
  constexpr float ROBOT_WIDTH = 0.85f;
  constexpr float ROBOT_DEPTH = 0.85f;
  constexpr float ROBOT_HEIGHT = 1.05f;

  visualization_msgs::msg::Marker bbox_marker;
  bbox_marker.header.stamp = stamp;
  bbox_marker.header.frame_id = target_frame_;
  bbox_marker.ns = "opponent_robot";
  bbox_marker.id = 0;
  bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
  bbox_marker.action = visualization_msgs::msg::Marker::ADD;
  bbox_marker.pose.position.x = pos.x();
  bbox_marker.pose.position.y = pos.y();
  bbox_marker.pose.position.z = ROBOT_HEIGHT * 0.5f;
  bbox_marker.pose.orientation.x = q_heading.x();
  bbox_marker.pose.orientation.y = q_heading.y();
  bbox_marker.pose.orientation.z = q_heading.z();
  bbox_marker.pose.orientation.w = q_heading.w();
  bbox_marker.scale.x = ROBOT_WIDTH;
  bbox_marker.scale.y = ROBOT_DEPTH;
  bbox_marker.scale.z = ROBOT_HEIGHT;
  bbox_marker.color.r = 1.0f;
  bbox_marker.color.g = 0.15f;
  bbox_marker.color.b = 0.15f;
  bbox_marker.color.a = 0.70f;
  markers.markers.push_back(bbox_marker);

  // 2. Opponent Heading Direction Arrow
  visualization_msgs::msg::Marker opp_head_marker;
  opp_head_marker.header.stamp = stamp;
  opp_head_marker.header.frame_id = target_frame_;
  opp_head_marker.ns = "opponent_heading";
  opp_head_marker.id = 5;
  opp_head_marker.type = visualization_msgs::msg::Marker::ARROW;
  opp_head_marker.action = visualization_msgs::msg::Marker::ADD;
  opp_head_marker.pose.position.x = pos.x();
  opp_head_marker.pose.position.y = pos.y();
  opp_head_marker.pose.position.z = ROBOT_HEIGHT + 0.10f;
  opp_head_marker.pose.orientation.x = q_heading.x();
  opp_head_marker.pose.orientation.y = q_heading.y();
  opp_head_marker.pose.orientation.z = q_heading.z();
  opp_head_marker.pose.orientation.w = q_heading.w();
  opp_head_marker.scale.x = 0.70;
  opp_head_marker.scale.y = 0.10;
  opp_head_marker.scale.z = 0.10;
  opp_head_marker.color.r = 1.0f;
  opp_head_marker.color.g = 0.4f;
  opp_head_marker.color.b = 0.0f;
  opp_head_marker.color.a = 0.95f;
  markers.markers.push_back(opp_head_marker);

  // 3. Mobile Bucket Cylinder Marker (100pt Target on top)
  visualization_msgs::msg::Marker bucket_marker;
  bucket_marker.header.stamp = stamp;
  bucket_marker.header.frame_id = target_frame_;
  bucket_marker.ns = "mobile_bucket";
  bucket_marker.id = 1;
  bucket_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  bucket_marker.action = visualization_msgs::msg::Marker::ADD;
  bucket_marker.pose.position.x = bucket.x();
  bucket_marker.pose.position.y = bucket.y();
  bucket_marker.pose.position.z = bucket.z();
  bucket_marker.pose.orientation.w = 1.0;
  bucket_marker.scale.x = 0.273;  // PO-24A diameter
  bucket_marker.scale.y = 0.273;
  bucket_marker.scale.z = 0.255;  // PO-24A height
  bucket_marker.color.r = 1.0f;
  bucket_marker.color.g = 0.85f;
  bucket_marker.color.b = 0.0f;
  bucket_marker.color.a = 0.85f;
  markers.markers.push_back(bucket_marker);

  // 4. Bright Green Aiming Target Sphere & Reticle (Cloth Launcher Aiming Point)
  visualization_msgs::msg::Marker aim_marker;
  aim_marker.header.stamp = stamp;
  aim_marker.header.frame_id = target_frame_;
  aim_marker.ns = "target_crosshair";
  aim_marker.id = 2;
  aim_marker.type = visualization_msgs::msg::Marker::SPHERE;
  aim_marker.action = visualization_msgs::msg::Marker::ADD;
  aim_marker.pose.position.x = bucket.x();
  aim_marker.pose.position.y = bucket.y();
  aim_marker.pose.position.z = bucket.z() + 0.15;
  aim_marker.pose.orientation.w = 1.0;
  aim_marker.scale.x = 0.28;
  aim_marker.scale.y = 0.28;
  aim_marker.scale.z = 0.28;
  aim_marker.color.r = 0.0f;
  aim_marker.color.g = 1.0f;
  aim_marker.color.b = 0.2f;
  aim_marker.color.a = 0.95f;
  markers.markers.push_back(aim_marker);

  // 5. 100pt Target Text Label
  visualization_msgs::msg::Marker target_label;
  target_label.header.stamp = stamp;
  target_label.header.frame_id = target_frame_;
  target_label.ns = "target_crosshair";
  target_label.id = 6;
  target_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  target_label.action = visualization_msgs::msg::Marker::ADD;
  target_label.pose.position.x = bucket.x();
  target_label.pose.position.y = bucket.y();
  target_label.pose.position.z = bucket.z() + 0.35;
  target_label.pose.orientation.w = 1.0;
  target_label.scale.z = 0.28;
  target_label.color.r = 0.1f;
  target_label.color.g = 1.0f;
  target_label.color.b = 0.1f;
  target_label.color.a = 1.0f;
  target_label.text = "[ 100pt BUCKET TARGET ]";
  markers.markers.push_back(target_label);

  // 6. Velocity Arrow
  if (speed > 0.1f)
  {
    visualization_msgs::msg::Marker vel_marker;
    vel_marker.header.stamp = stamp;
    vel_marker.header.frame_id = target_frame_;
    vel_marker.ns = "velocity_vector";
    vel_marker.id = 3;
    vel_marker.type = visualization_msgs::msg::Marker::ARROW;
    vel_marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start_pt;
    start_pt.x = pos.x();
    start_pt.y = pos.y();
    start_pt.z = 0.8;

    geometry_msgs::msg::Point end_pt;
    end_pt.x = pos.x() + vel.x();
    end_pt.y = pos.y() + vel.y();
    end_pt.z = 0.8;

    vel_marker.points.push_back(start_pt);
    vel_marker.points.push_back(end_pt);
    vel_marker.scale.x = 0.06;  // shaft diameter
    vel_marker.scale.y = 0.12;  // head diameter
    vel_marker.scale.z = 0.15;  // head length
    vel_marker.color.r = 0.2f;
    vel_marker.color.g = 0.8f;
    vel_marker.color.b = 1.0f;
    vel_marker.color.a = 0.9f;
    markers.markers.push_back(vel_marker);
  }

  // 7. Status Text (HUD)
  visualization_msgs::msg::Marker text_marker;
  text_marker.header.stamp = stamp;
  text_marker.header.frame_id = target_frame_;
  text_marker.ns = "status_text";
  text_marker.id = 4;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose.position.x = pos.x();
  text_marker.pose.position.y = pos.y();
  text_marker.pose.position.z = bucket.z() + 0.65;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.z = 0.22;  // Text size
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0f;

  char buf[128];
  std::snprintf(buf, sizeof(buf), "OPPONENT [TRACKING]\nV=%.2fm/s | Z_b=%.2fm", speed, bucket.z());
  text_marker.text = std::string(buf);
  markers.markers.push_back(text_marker);

  pub_markers_->publish(markers);
}

void OpponentTrackerNode::publishOurRobotMarker(const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray markers;

  // 1. Our Robot Chassis Box (in base_link frame)
  visualization_msgs::msg::Marker body_marker;
  body_marker.header.stamp = stamp;
  body_marker.header.frame_id = "base_link";
  body_marker.ns = "our_robot";
  body_marker.id = 100;
  body_marker.type = visualization_msgs::msg::Marker::CUBE;
  body_marker.action = visualization_msgs::msg::Marker::ADD;
  body_marker.pose.position.x = 0.0;
  body_marker.pose.position.y = 0.0;
  body_marker.pose.position.z = 0.25;
  body_marker.pose.orientation.w = 1.0;
  body_marker.scale.x = 0.75;
  body_marker.scale.y = 0.75;
  body_marker.scale.z = 0.50;
  body_marker.color.r = 0.0f;
  body_marker.color.g = 0.7f;
  body_marker.color.b = 1.0f;
  body_marker.color.a = 0.75f;
  markers.markers.push_back(body_marker);

  // 2. Heading Forward Arrow
  visualization_msgs::msg::Marker head_marker;
  head_marker.header.stamp = stamp;
  head_marker.header.frame_id = "base_link";
  head_marker.ns = "our_heading";
  head_marker.id = 101;
  head_marker.type = visualization_msgs::msg::Marker::ARROW;
  head_marker.action = visualization_msgs::msg::Marker::ADD;
  head_marker.pose.position.x = 0.0;
  head_marker.pose.position.y = 0.0;
  head_marker.pose.position.z = 0.55;
  head_marker.pose.orientation.w = 1.0;
  head_marker.scale.x = 0.60; // length
  head_marker.scale.y = 0.08; // width
  head_marker.scale.z = 0.08; // height
  head_marker.color.r = 0.0f;
  head_marker.color.g = 1.0f;
  head_marker.color.b = 0.3f;
  head_marker.color.a = 0.95f;
  markers.markers.push_back(head_marker);

  // 3. Name Label
  visualization_msgs::msg::Marker name_marker;
  name_marker.header.stamp = stamp;
  name_marker.header.frame_id = "base_link";
  name_marker.ns = "our_label";
  name_marker.id = 102;
  name_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  name_marker.action = visualization_msgs::msg::Marker::ADD;
  name_marker.pose.position.x = 0.0;
  name_marker.pose.position.y = 0.0;
  name_marker.pose.position.z = 0.85;
  name_marker.pose.orientation.w = 1.0;
  name_marker.scale.z = 0.25;
  name_marker.color.r = 0.2f;
  name_marker.color.g = 0.9f;
  name_marker.color.b = 1.0f;
  name_marker.color.a = 1.0f;
  name_marker.text = "OUR ROBOT (A-TEAM)";
  markers.markers.push_back(name_marker);

  pub_our_robot_->publish(markers);
}

}  // namespace opponent_tracker
