#ifndef CHIKURIN_CONTROL_BODY_TRACKER_SCAN_CONTROLLER_HPP
#define CHIKURIN_CONTROL_BODY_TRACKER_SCAN_CONTROLLER_HPP

#include "chikurin_control/body_tracker/params.hpp"
#include <cmath>

namespace body_tracker {

enum class ScanDirection {
  RIGHT,
  LEFT
};

struct ScanVelocity {
  double pan = 0.0;
  double tilt = 0.0;
};

class ScanController {
public:
  explicit ScanController(const ScanParams& params);

  ScanVelocity calculate(int current_pan_counts, int current_tilt_counts);
  void reset();

  bool isEnabled() const { return params_.enabled; }
  ScanDirection getDirection() const { return direction_; }
  int getTargetTiltCounts() const { return target_tilt_counts_; }

  static int radiansToEncoderCounts(double radians, double counts_per_revolution) {
    return static_cast<int>(radians * counts_per_revolution / (2.0 * M_PI));
  }

  static double encoderCountsToRadians(int counts, double counts_per_revolution) {
    return static_cast<double>(counts) * 2.0 * M_PI / counts_per_revolution;
  }

private:
  ScanParams params_;
  ScanDirection direction_ = ScanDirection::RIGHT;
  int target_tilt_counts_ = 0;
};

}  // namespace body_tracker

#endif  // CHIKURIN_CONTROL_BODY_TRACKER_SCAN_CONTROLLER_HPP
