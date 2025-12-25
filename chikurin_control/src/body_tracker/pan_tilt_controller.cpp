#include "chikurin_control/body_tracker/pan_tilt_controller.hpp"
#include <algorithm>
#include <cmath>

namespace {
template<typename T>
T clamp(T val, T lo, T hi) {
  return std::max(lo, std::min(val, hi));
}
}

namespace body_tracker {

PanTiltController::PanTiltController(const PanTiltParams& params)
  : params_(params)
{
}

ControlOutput PanTiltController::compute(int body_center_x, int body_center_y,
                                         int frame_center_x, int frame_center_y,
                                         double current_pan, double current_tilt)
{
  ControlOutput output;

  // Calculate target velocity (pixels/frame -> normalized)
  double target_vel_x = 0.0;
  double target_vel_y = 0.0;
  if (prev_body_center_x_ >= 0 && prev_body_center_y_ >= 0) {
    target_vel_x = static_cast<double>(body_center_x - prev_body_center_x_) / frame_center_x;
    target_vel_y = static_cast<double>(body_center_y - prev_body_center_y_) / frame_center_y;
  }

  // Calculate errors (normalized)
  output.pan_error = static_cast<double>(frame_center_x - body_center_x) / frame_center_x;
  output.tilt_error = static_cast<double>(frame_center_y - body_center_y) / frame_center_y;

  // PI control for pan
  pan_integral_error_ += output.pan_error;
  double pan_max_integral = params_.pan_limit / (params_.pan_ki + 1e-6);
  pan_integral_error_ = clamp(pan_integral_error_, -pan_max_integral, pan_max_integral);
  output.pan_p_term = params_.pan_kp * output.pan_error;
  output.pan_i_term = params_.pan_ki * pan_integral_error_;
  double pan_delta = output.pan_p_term + output.pan_i_term;

  // PI control for tilt
  tilt_integral_error_ += output.tilt_error;
  double tilt_max_integral = params_.tilt_limit / (params_.tilt_ki + 1e-6);
  tilt_integral_error_ = clamp(tilt_integral_error_, -tilt_max_integral, tilt_max_integral);
  output.tilt_p_term = params_.tilt_kp * output.tilt_error;
  output.tilt_i_term = params_.tilt_ki * tilt_integral_error_;
  double tilt_delta = -(output.tilt_p_term + output.tilt_i_term);

  // Velocity feedforward
  double pan_feedforward = -params_.velocity_feedforward_gain * target_vel_x;
  double tilt_feedforward = -params_.velocity_feedforward_gain * target_vel_y;

  // Set velocity commands
  output.pan_velocity = pan_delta + pan_feedforward;
  output.tilt_velocity = tilt_delta + tilt_feedforward;

  // Calculate base angular velocity
  double pan_ratio = std::abs(current_pan) / params_.pan_limit;
  if (pan_ratio > params_.pan_base_threshold) {
    double excess_ratio = (pan_ratio - params_.pan_base_threshold) /
                          (1.0 - params_.pan_base_threshold);
    output.base_angular_velocity = std::copysign(excess_ratio * params_.base_angular_gain, current_pan);
  }

  return output;
}

void PanTiltController::reset()
{
  pan_integral_error_ = 0.0;
  tilt_integral_error_ = 0.0;
  prev_body_center_x_ = -1;
  prev_body_center_y_ = -1;
}

void PanTiltController::decay(double factor)
{
  pan_integral_error_ *= factor;
  tilt_integral_error_ *= factor;
}

}  // namespace body_tracker
