#include "chikurin_control/body_tracker/scan_controller.hpp"
#include <algorithm>

namespace {
template<typename T>
T clamp(T val, T lo, T hi) {
  return std::max(lo, std::min(val, hi));
}
}

namespace body_tracker {

ScanController::ScanController(const ScanParams& params)
  : params_(params)
  , target_tilt_counts_(params_.tilt_min_counts)
{
}

ScanVelocity ScanController::calculate(int current_pan_counts, int current_tilt_counts)
{
  ScanVelocity velocity;

  if (!params_.enabled) {
    return velocity;
  }

  // Check if reached pan limits and switch direction
  if (direction_ == ScanDirection::RIGHT) {
    if (current_pan_counts >= params_.pan_max_counts) {
      direction_ = ScanDirection::LEFT;
      target_tilt_counts_ += params_.tilt_step_counts;
      if (target_tilt_counts_ > params_.tilt_max_counts) {
        target_tilt_counts_ = params_.tilt_min_counts;
      }
    }
    velocity.pan = params_.pan_velocity;
  } else {
    if (current_pan_counts <= params_.pan_min_counts) {
      direction_ = ScanDirection::RIGHT;
      target_tilt_counts_ += params_.tilt_step_counts;
      if (target_tilt_counts_ > params_.tilt_max_counts) {
        target_tilt_counts_ = params_.tilt_min_counts;
      }
    }
    velocity.pan = -params_.pan_velocity;
  }

  // Tilt: P-control towards target (encoder counts -> rad/s output)
  int tilt_error_counts = target_tilt_counts_ - current_tilt_counts;
  double tilt_error_rad = encoderCountsToRadians(tilt_error_counts, params_.tilt_counts_per_revolution);
  velocity.tilt = clamp(tilt_error_rad * 2.0, -params_.tilt_velocity, params_.tilt_velocity);

  return velocity;
}

void ScanController::reset()
{
  direction_ = ScanDirection::RIGHT;
  target_tilt_counts_ = params_.tilt_min_counts;
}

}  // namespace body_tracker
