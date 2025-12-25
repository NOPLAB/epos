#ifndef CHIKURIN_CONTROL_BODY_TRACKER_FIRING_CONTROLLER_HPP
#define CHIKURIN_CONTROL_BODY_TRACKER_FIRING_CONTROLLER_HPP

#include <chrono>
#include <functional>
#include <cmath>

#include "chikurin_control/body_tracker/params.hpp"

namespace body_tracker {

enum class FiringState {
  IDLE,
  SPINUP,
  FIRING,
  COOLDOWN
};

class FiringController {
public:
  using ShooterVelocityCallback = std::function<void(double)>;
  using TriggerCallback = std::function<void()>;

  explicit FiringController(const AutoFireParams& params);

  void setCallbacks(ShooterVelocityCallback shooter_cb, TriggerCallback trigger_cb) {
    shooter_callback_ = shooter_cb;
    trigger_callback_ = trigger_cb;
  }

  void update();

  bool tryFire(float confidence, double pan_error, double tilt_error);

  void stop();

  bool isEnabled() const { return params_.enabled; }
  FiringState getState() const { return state_; }
  bool isIdle() const { return state_ == FiringState::IDLE; }

private:
  AutoFireParams params_;
  FiringState state_ = FiringState::IDLE;
  std::chrono::steady_clock::time_point state_start_time_;

  ShooterVelocityCallback shooter_callback_;
  TriggerCallback trigger_callback_;
};

}  // namespace body_tracker

#endif  // CHIKURIN_CONTROL_BODY_TRACKER_FIRING_CONTROLLER_HPP
