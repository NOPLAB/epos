#include "chikurin_control/body_tracker/firing_controller.hpp"

namespace body_tracker {

FiringController::FiringController(const AutoFireParams& params)
  : params_(params)
{
}

void FiringController::update()
{
  if (!params_.enabled) {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - state_start_time_);

  switch (state_) {
    case FiringState::IDLE:
      // Nothing to do in IDLE
      break;

    case FiringState::SPINUP:
      if (elapsed.count() >= params_.spinup_ms) {
        // Spinup complete, trigger fire
        if (trigger_callback_) {
          trigger_callback_();
        }
        state_ = FiringState::FIRING;
        state_start_time_ = now;
      }
      break;

    case FiringState::FIRING:
      if (elapsed.count() >= params_.fire_duration_ms) {
        // Stop shooter
        if (shooter_callback_) {
          shooter_callback_(0.0);
        }
        state_ = FiringState::COOLDOWN;
        state_start_time_ = now;
      }
      break;

    case FiringState::COOLDOWN:
      if (elapsed.count() >= params_.cooldown_ms) {
        state_ = FiringState::IDLE;
      }
      break;
  }
}

bool FiringController::tryFire(float confidence, double pan_error, double tilt_error)
{
  if (!params_.enabled) {
    return false;
  }

  if (state_ != FiringState::IDLE) {
    return false;
  }

  if (confidence < params_.threshold) {
    return false;
  }

  if (std::abs(pan_error) > params_.center_tolerance ||
      std::abs(tilt_error) > params_.center_tolerance) {
    return false;
  }

  // Start firing sequence
  if (shooter_callback_) {
    shooter_callback_(params_.getShooterSpeedRadS());
  }

  state_ = FiringState::SPINUP;
  state_start_time_ = std::chrono::steady_clock::now();

  return true;
}

void FiringController::stop()
{
  if (state_ != FiringState::IDLE && shooter_callback_) {
    shooter_callback_(0.0);
  }
  state_ = FiringState::IDLE;
}

}  // namespace body_tracker
