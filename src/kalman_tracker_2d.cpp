#include "opponent_tracker/kalman_tracker_2d.hpp"

#include <algorithm>
#include <cmath>

namespace opponent_tracker {

KalmanTracker2D::KalmanTracker2D(const TrackerParams& params) : params_(params)
{
    reset();
}

void KalmanTracker2D::setParams(const TrackerParams& params)
{
    params_ = params;
}

const TrackerParams& KalmanTracker2D::getParams() const
{
    return params_;
}

void KalmanTracker2D::setStaticObstacles(const std::vector<StaticObstacle>& obstacles)
{
    static_obstacles_ = obstacles;
}

void KalmanTracker2D::reset()
{
    state_ = TrackerState::UNINITIALIZED;
    x_.setZero();
    P_.setIdentity();
    P_ *= 1.0f;
    smooth_dims_    = Eigen::Vector3f(0.85f, 0.85f, 1.20f);
    smooth_heading_ = 0.0f;
    hits_           = 0;
    missed_count_   = 0;
}

void KalmanTracker2D::applyCollisionConstraints()
{
    // 1. Prevent penetrating static vertical obstacles (posts, tall objects)
    constexpr float ROBOT_RADIUS = 0.40f;  // Half-size of opponent chassis
    for (const auto& obs : static_obstacles_) {
        if (obs.name == "CenterBarrier") {
            continue;  // Center barrier is a low floor divider (15cm)
        }

        const float half_x = obs.size_xy.x() * 0.5f + ROBOT_RADIUS;
        const float half_y = obs.size_xy.y() * 0.5f + ROBOT_RADIUS;

        const float dx = x_(0) - obs.center_xy.x();
        const float dy = x_(1) - obs.center_xy.y();

        if (std::abs(dx) < half_x && std::abs(dy) < half_y) {
            const float push_x = half_x - std::abs(dx);
            const float push_y = half_y - std::abs(dy);

            if (push_x < push_y) {
                x_(0) = obs.center_xy.x() + (dx >= 0.0f ? half_x : -half_x);
                x_(2) = 0.0f;
            } else {
                x_(1) = obs.center_xy.y() + (dy >= 0.0f ? half_y : -half_y);
                x_(3) = 0.0f;
            }
        }
    }

    // 2. Field boundary constraints [-4.80m, 4.80m] x [-5.25m, 5.25m]
    x_(0) = std::clamp(x_(0), -4.80f, 4.80f);
    x_(1) = std::clamp(x_(1), -5.25f, 5.25f);

    // 3. Velocity limit clamping (Max 2.5 m/s) to prevent teleportation
    const float speed         = std::sqrt(x_(2) * x_(2) + x_(3) * x_(3));
    constexpr float MAX_SPEED = 2.5f;
    if (speed > MAX_SPEED) {
        const float scale = MAX_SPEED / speed;
        x_(2) *= scale;
        x_(3) *= scale;
    }
}

void KalmanTracker2D::predict(double dt)
{
    if (state_ == TrackerState::UNINITIALIZED || state_ == TrackerState::LOST) {
        return;
    }

    const float f_dt = static_cast<float>(dt);

    // State Transition Matrix F (5x5)
    Eigen::Matrix<float, 5, 5> F = Eigen::Matrix<float, 5, 5>::Identity();
    F(0, 2)                      = f_dt;
    F(1, 3)                      = f_dt;

    // Process Noise Covariance Q (5x5)
    Eigen::Matrix<float, 5, 5> Q = Eigen::Matrix<float, 5, 5>::Zero();
    const float dt2              = f_dt * f_dt;
    const float dt3              = dt2 * f_dt / 2.0f;
    const float dt4              = dt2 * dt2 / 4.0f;
    const float q_pos            = params_.process_noise_pos;
    const float q_vel            = params_.process_noise_vel;

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

    applyCollisionConstraints();
}

void KalmanTracker2D::update(
    const Eigen::Vector3f& robot_pos,
    const Eigen::Vector3f& bucket_pos,
    const Eigen::Vector3f& raw_dims
)
{
    // 1. Smooth dimensions using Exponential Moving Average (EMA) to prevent jitter/morphing
    Eigen::Vector3f clamped_dims = raw_dims;
    clamped_dims.x()             = std::clamp(clamped_dims.x(), 0.70f, 1.10f);
    clamped_dims.y()             = std::clamp(clamped_dims.y(), 0.70f, 1.10f);
    clamped_dims.z()             = std::clamp(clamped_dims.z(), 0.90f, 1.60f);

    constexpr float DIM_ALPHA = 0.85f;  // 85% previous, 15% new (smooth transition)
    smooth_dims_              = DIM_ALPHA * smooth_dims_ + (1.0f - DIM_ALPHA) * clamped_dims;

    if (state_ == TrackerState::UNINITIALIZED || state_ == TrackerState::LOST) {
        // Initialize state
        x_(0) = robot_pos.x();
        x_(1) = robot_pos.y();
        x_(2) = 0.0f;
        x_(3) = 0.0f;
        x_(4) = std::clamp(bucket_pos.z(), 1.20f, 2.10f);

        P_.setIdentity();
        P_(0, 0) = params_.measurement_noise_pos;
        P_(1, 1) = params_.measurement_noise_pos;
        P_(2, 2) = 0.5f;  // Tighter initial velocity uncertainty to prevent sudden velocity jump
        P_(3, 3) = 0.5f;
        P_(4, 4) = params_.measurement_noise_z;

        smooth_dims_    = clamped_dims;  // Immediately adopt initial detection size
        hits_           = 1;
        missed_count_   = 0;
        smooth_heading_ = 0.0f;
        state_ =
            (params_.min_hits_to_confirm <= 1) ? TrackerState::TRACKING : TrackerState::COASTING;
        applyCollisionConstraints();
        return;
    }

    // If in COASTING (unconfirmed) and new measurement is too far, reset to new measurement
    if (state_ == TrackerState::COASTING) {
        const float dist = (robot_pos.head<2>() - x_.head<2>()).norm();
        if (dist > 1.0f) {
            // Previous hit was likely a transient noise or initial TF jump -> reset tracker here
            x_(0)         = robot_pos.x();
            x_(1)         = robot_pos.y();
            x_(2)         = 0.0f;
            x_(3)         = 0.0f;
            x_(4)         = std::clamp(bucket_pos.z(), 1.20f, 2.10f);
            smooth_dims_  = clamped_dims;
            hits_         = 1;
            missed_count_ = 0;
            applyCollisionConstraints();
            return;
        }
    }

    // Measurement Vector z = [x_meas, y_meas, z_bucket_meas]^T (3x1)
    Eigen::Matrix<float, 3, 1> z;
    z << robot_pos.x(), robot_pos.y(), std::clamp(bucket_pos.z(), 1.20f, 2.10f);

    // Measurement Matrix H (3x5)
    Eigen::Matrix<float, 3, 5> H = Eigen::Matrix<float, 3, 5>::Zero();
    H(0, 0)                      = 1.0f;
    H(1, 1)                      = 1.0f;
    H(2, 4)                      = 1.0f;

    // Measurement Noise Covariance R (3x3)
    Eigen::Matrix<float, 3, 3> R = Eigen::Matrix<float, 3, 3>::Identity();
    R(0, 0)                      = params_.measurement_noise_pos * params_.measurement_noise_pos;
    R(1, 1)                      = params_.measurement_noise_pos * params_.measurement_noise_pos;
    R(2, 2)                      = params_.measurement_noise_z * params_.measurement_noise_z;

    // Innovation / Residual
    const Eigen::Matrix<float, 3, 1> y = z - H * x_;
    const Eigen::Matrix<float, 3, 3> S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<float, 5, 3> K = P_ * H.transpose() * S.inverse();

    // Update State and Covariance
    x_                                 = x_ + K * y;
    const Eigen::Matrix<float, 5, 5> I = Eigen::Matrix<float, 5, 5>::Identity();
    P_                                 = (I - K * H) * P_;

    // Suppress initial velocity spike during the first few frames
    if (hits_ < 4) {
        x_(2) *= 0.5f;
        x_(3) *= 0.5f;
    }

    applyCollisionConstraints();

    // 2. Smooth heading angle (angular EMA) and zero residual velocity noise when stationary
    const float current_speed = getSpeed();
    if (current_speed > 0.28f) {
        const float raw_yaw = std::atan2(x_(3), x_(2));
        float diff          = raw_yaw - smooth_heading_;
        while (diff > static_cast<float>(M_PI)) diff -= 2.0f * static_cast<float>(M_PI);
        while (diff < -static_cast<float>(M_PI)) diff += 2.0f * static_cast<float>(M_PI);
        constexpr float HEADING_ALPHA = 0.90f;
        smooth_heading_ += (1.0f - HEADING_ALPHA) * diff;
    } else {
        // Stationary: suppress LiDAR centroid jitter velocity
        x_(2) = 0.0f;
        x_(3) = 0.0f;
    }

    ++hits_;
    missed_count_ = 0;
    if (hits_ >= params_.min_hits_to_confirm) {
        state_ = TrackerState::TRACKING;
    }
}

void KalmanTracker2D::markMissed()
{
    if (state_ == TrackerState::UNINITIALIZED || state_ == TrackerState::LOST) {
        return;
    }

    ++missed_count_;
    if (missed_count_ > params_.max_missed_frames) {
        state_ = TrackerState::LOST;
        hits_  = 0;
    } else {
        state_ = TrackerState::COASTING;
    }
}

TrackerState KalmanTracker2D::getState() const
{
    return state_;
}

bool KalmanTracker2D::isTracking() const
{
    return (state_ == TrackerState::TRACKING);
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
    return smooth_heading_;
}

Eigen::Vector3f KalmanTracker2D::getBucketTarget() const
{
    return Eigen::Vector3f(x_(0), x_(1), x_(4));
}

Eigen::Vector3f KalmanTracker2D::getDimensions() const
{
    return smooth_dims_;
}

float KalmanTracker2D::computeDistance(const Eigen::Vector3f& measurement) const
{
    if (state_ == TrackerState::UNINITIALIZED || state_ == TrackerState::LOST) {
        return 0.0f;
    }
    const float dx = measurement.x() - x_(0);
    const float dy = measurement.y() - x_(1);
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace opponent_tracker
