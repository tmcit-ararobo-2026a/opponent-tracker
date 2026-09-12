#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "opponent_tracker/cluster_extractor.hpp"

namespace opponent_tracker
{

enum class TrackerState
{
  UNINITIALIZED,
  TRACKING,
  COASTING,
  LOST
};

struct TrackerParams
{
  float process_noise_pos{2.0f};      // Acceleration / position process noise [m/s^2]
  float process_noise_vel{5.0f};      // Velocity process noise
  float process_noise_z{0.20f};       // Bucket height process noise
  float measurement_noise_pos{0.02f}; // LiDAR position measurement noise [m]
  float measurement_noise_z{0.02f};   // Bucket height measurement noise [m]
  float max_association_dist{2.0f};   // Max Euclidean distance to associate measurement [m]
  int max_missed_frames{10};          // Max missed frames before losing track (~0.5s at 20Hz)
  int min_hits_to_confirm{2};         // Consecutive detections to confirm tracking
};

class KalmanTracker2D
{
public:
  explicit KalmanTracker2D(const TrackerParams & params = TrackerParams());

  void setParams(const TrackerParams & params);
  const TrackerParams & getParams() const;

  void reset();

  // Predict step given elapsed time dt [seconds]
  void predict(double dt);

  // Update step given measured robot centroid (x, y), bucket center (x_b, y_b, z_b), and measured bbox dimensions
  void update(const Eigen::Vector3f & robot_pos, const Eigen::Vector3f & bucket_pos, const Eigen::Vector3f & raw_dims);

  // Mark a missed detection frame
  void markMissed();

  // Set obstacles to prevent robot from penetrating boxes/desks
  void setStaticObstacles(const std::vector<StaticObstacle> & obstacles);

  // State queries
  TrackerState getState() const;
  bool isTracking() const;

  Eigen::Vector2f getPosition() const;      // [x, y] in map frame
  Eigen::Vector2f getVelocity() const;      // [vx, vy] in map frame
  float getSpeed() const;                  // Speed in m/s
  float getHeading() const;                // Yaw angle in radians [-pi, pi] (smoothed)
  Eigen::Vector3f getBucketTarget() const; // [x, y, z] target for cloth launcher
  Eigen::Vector3f getDimensions() const;   // [width, depth, height] (smoothed & stable)

  // Distance gating to check if measurement belongs to this tracker
  float computeDistance(const Eigen::Vector3f & measurement) const;

private:
  void applyCollisionConstraints();

  TrackerParams params_;
  TrackerState state_{TrackerState::UNINITIALIZED};
  std::vector<StaticObstacle> static_obstacles_;

  // State: [x, y, vx, vy, z_bucket]^T
  Eigen::Matrix<float, 5, 1> x_;
  // Covariance matrix 5x5
  Eigen::Matrix<float, 5, 5> P_;

  // Smooth stable dimensions & heading
  Eigen::Vector3f smooth_dims_{0.85f, 0.85f, 1.20f};
  float smooth_heading_{0.0f};

  int hits_{0};
  int missed_count_{0};
};

}  // namespace opponent_tracker

