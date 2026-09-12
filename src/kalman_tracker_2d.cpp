#include "opponent_tracker/kalman_tracker_2d.hpp"

#include <cmath>

namespace opponent_tracker
{

KalmanTracker2D::KalmanTracker2D(const TrackerParams & params)
: params_(params)
{
  reset();
}

void KalmanTracker2D::setParams(const TrackerParams & params)
{
  params_ = params;
}

const TrackerParams & KalmanTracker2D::getParams() const
{
  return params_;
}

void KalmanTracker2D::reset()
{
  state_ = TrackerState::UNINITIALIZED;
  x_.setZero();
  P_.setIdentity();
  P_ *= 1.0f;
  hits_ = 0;
  missed_count_ = 0;
}

void KalmanTracker2D::predict(double dt)
{
  if (state_ == TrackerState::UNINITIALIZED || state_ == TrackerState::LOST)
  {
    return;
  }

  const float f_dt = static_cast<float>(dt);

  // State Transition Matrix F (5x5)
  // [x, y, vx, vy, z_b]
  Eigen::Matrix<float, 5, 5> F = Eigen::Matrix<float, 5, 5>::Identity();
  F(0, 2) = f_dt;
  F(1, 3) = f_dt;

  // Process Noise Covariance Q (5x5)
  Eigen::Matrix<float, 5, 5> Q = Eigen::Matrix<float, 5, 5>::Zero();
  const float dt2 = f_dt * f_dt;
  const float dt3 = dt2 * f_dt / 2.0f;
  const float dt4 = dt2 * dt2 / 4.0f;
  const float q_pos = params_.process_noise_pos;
  const float q_vel = params_.process_noise_vel;

  Q(0, 0) = dt4 * q_vel + dt2 * q_pos;
  Q(0, 2) = dt3 * q_vel;
  Q(2, 0) = dt3 * q_vel;
  Q(2, 2) = dt2 * q_vel;

  Q(1, 1) = dt4 * q_vel + dt2 * q_pos;
  Q(1, 3) = dt3 * q_vel;
  Q(3, 1) = dt3 * q_vel;
  Q(3, 3) = dt2 * q_vel;

  Q(4, 4) = dt2 * params_.process_noise_z;

  // Predict State and Covariance
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanTracker2D::update(const Eigen::Vector3f & robot_pos, const Eigen::Vector3f & bucket_pos)
{
  if (state_ == TrackerState::UNINITIALIZED || state_ == TrackerState::LOST)
  {
    // Initialize state
    x_(0) = robot_pos.x();
    x_(1) = robot_pos.y();
    x_(2) = 0.0f;
    x_(3) = 0.0f;
    x_(4) = bucket_pos.z();

    P_.setIdentity();
    P_(0, 0) = params_.measurement_noise_pos;
    P_(1, 1) = params_.measurement_noise_pos;
    P_(2, 2) = 1.0f;
    P_(3, 3) = 1.0f;
    P_(4, 4) = params_.measurement_noise_z;

    hits_ = 1;
    missed_count_ = 0;
    state_ = (params_.min_hits_to_confirm <= 1) ? TrackerState::TRACKING : TrackerState::COASTING;
    return;
  }

  // Measurement Vector z = [x_meas, y_meas, z_bucket_meas]^T (3x1)
  Eigen::Matrix<float, 3, 1> z;
  z << robot_pos.x(), robot_pos.y(), bucket_pos.z();

  // Measurement Matrix H (3x5)
  Eigen::Matrix<float, 3, 5> H = Eigen::Matrix<float, 3, 5>::Zero();
  H(0, 0) = 1.0f;
  H(1, 1) = 1.0f;
  H(2, 4) = 1.0f;

  // Measurement Noise Covariance R (3x3)
  Eigen::Matrix<float, 3, 3> R = Eigen::Matrix<float, 3, 3>::Identity();
  R(0, 0) = params_.measurement_noise_pos * params_.measurement_noise_pos;
  R(1, 1) = params_.measurement_noise_pos * params_.measurement_noise_pos;
  R(2, 2) = params_.measurement_noise_z * params_.measurement_noise_z;

  // Innovation / Residual
  const Eigen::Matrix<float, 3, 1> y = z - H * x_;
  const Eigen::Matrix<float, 3, 3> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<float, 5, 3> K = P_ * H.transpose() * S.inverse();

  // Update State and Covariance
  x_ = x_ + K * y;
  const Eigen::Matrix<float, 5, 5> I = Eigen::Matrix<float, 5, 5>::Identity();
  P_ = (I - K * H) * P_;

  ++hits_;
  missed_count_ = 0;
  if (hits_ >= params_.min_hits_to_confirm)
  {
    state_ = TrackerState::TRACKING;
  }
}

void KalmanTracker2D::markMissed()
{
  if (state_ == TrackerState::UNINITIALIZED || state_ == TrackerState::LOST)
  {
    return;
  }

  ++missed_count_;
  if (missed_count_ > params_.max_missed_frames)
  {
    state_ = TrackerState::LOST;
    hits_ = 0;
  }
  else
  {
    state_ = TrackerState::COASTING;
  }
}

TrackerState KalmanTracker2D::getState() const
{
  return state_;
}

bool KalmanTracker2D::isTracking() const
{
  return (state_ == TrackerState::TRACKING || state_ == TrackerState::COASTING);
}

Eigen::Vector2f KalmanTracker2D::getPosition() const
{
  return Eigen::Vector2f(x_(0), x_(1));
}

Eigen::Vector2f KalmanTracker2D::getVelocity() const
{
  return Eigen::Vector2f(x_(2), x_(3));
}

float KalmanTracker2D::getSpeed() const
{
  return std::sqrt(x_(2) * x_(2) + x_(3) * x_(3));
}

float KalmanTracker2D::getHeading() const
{
  if (getSpeed() < 0.05f)
  {
    return 0.0f;
  }
  return std::atan2(x_(3), x_(2));
}

Eigen::Vector3f KalmanTracker2D::getBucketTarget() const
{
  return Eigen::Vector3f(x_(0), x_(1), x_(4));
}

float KalmanTracker2D::computeDistance(const Eigen::Vector3f & measurement) const
{
  if (state_ == TrackerState::UNINITIALIZED || state_ == TrackerState::LOST)
  {
    return 0.0f;
  }
  const float dx = measurement.x() - x_(0);
  const float dy = measurement.y() - x_(1);
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace opponent_tracker

