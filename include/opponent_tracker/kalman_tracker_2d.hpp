#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>

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
  float process_noise_pos{0.1f};      // Acceleration / position process noise [m/s^2]
  float process_noise_vel{1.5f};      // Velocity process noise
  float process_noise_z{0.05f};       // Bucket height process noise
  float measurement_noise_pos{0.05f}; // LiDAR position measurement noise [m]
  float measurement_noise_z{0.05f};   // Bucket height measurement noise [m]
  float max_association_dist{1.5f};   // Max Euclidean distance to associate measurement [m]
  int max_missed_frames{10};          // Max missed frames before losing track (~0.5s at 20Hz)
  int min_hits_to_confirm{3};         // Consecutive detections to confirm tracking
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

  // Update step given measured robot centroid (x, y) and bucket center (x_b, y_b, z_b)
  void update(const Eigen::Vector3f & robot_pos, const Eigen::Vector3f & bucket_pos);

  // Mark a missed detection frame
  void markMissed();

  // State queries
  TrackerState getState() const;
  bool isTracking() const;

  Eigen::Vector2f getPosition() const;      // [x, y] in map frame
  Eigen::Vector2f getVelocity() const;      // [vx, vy] in map frame
  float getSpeed() const;                  // Speed in m/s
  float getHeading() const;                // Yaw angle in radians [-pi, pi]
  Eigen::Vector3f getBucketTarget() const; // [x, y, z] target for cloth launcher

  // Distance gating to check if measurement belongs to this tracker
  float computeDistance(const Eigen::Vector3f & measurement) const;

private:
  TrackerParams params_;
  TrackerState state_{TrackerState::UNINITIALIZED};

  // State: [x, y, vx, vy, z_bucket]^T
  Eigen::Matrix<float, 5, 1> x_;
  // Covariance matrix 5x5
  Eigen::Matrix<float, 5, 5> P_;

  int hits_{0};
  int missed_count_{0};
};

}  // namespace opponent_tracker

