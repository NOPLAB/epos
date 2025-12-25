#ifndef CHIKURIN_CONTROL_BODY_TRACKER_PAN_TILT_CONTROLLER_HPP
#define CHIKURIN_CONTROL_BODY_TRACKER_PAN_TILT_CONTROLLER_HPP

#include "chikurin_control/body_tracker/params.hpp"

namespace body_tracker {

struct ControlOutput {
  double pan_velocity = 0.0;
  double tilt_velocity = 0.0;
  double pan_error = 0.0;
  double tilt_error = 0.0;
  double pan_p_term = 0.0;
  double pan_i_term = 0.0;
  double tilt_p_term = 0.0;
  double tilt_i_term = 0.0;
  double base_angular_velocity = 0.0;
};

class PanTiltController {
public:
  explicit PanTiltController(const PanTiltParams& params);

  ControlOutput compute(int body_center_x, int body_center_y,
                        int frame_center_x, int frame_center_y,
                        double current_pan, double current_tilt);

  void reset();
  void decay(double factor = 0.95);

  void updatePrevCenter(int x, int y) {
    prev_body_center_x_ = x;
    prev_body_center_y_ = y;
  }

  const PanTiltParams& getParams() const { return params_; }

private:
  PanTiltParams params_;

  double pan_integral_error_ = 0.0;
  double tilt_integral_error_ = 0.0;

  int prev_body_center_x_ = -1;
  int prev_body_center_y_ = -1;
};

}  // namespace body_tracker

#endif  // CHIKURIN_CONTROL_BODY_TRACKER_PAN_TILT_CONTROLLER_HPP
