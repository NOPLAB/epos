#ifndef CHIKURIN_CONTROL_BODY_TRACKER_PARAMS_HPP
#define CHIKURIN_CONTROL_BODY_TRACKER_PARAMS_HPP

#include <string>
#include <cmath>

namespace body_tracker {

struct PanTiltParams {
  double pan_kp = 0.002;
  double pan_ki = 0.0001;
  double tilt_kp = 0.002;
  double tilt_ki = 0.0001;
  double pan_limit = M_PI;
  double tilt_limit = M_PI / 2;
  double pan_base_threshold = 0.7;
  double base_angular_gain = 0.5;
  double velocity_feedforward_gain = 0.005;
};

struct DetectionParams {
  double confidence_threshold = 0.5;
  double nms_threshold = 0.4;
  int detection_skip_frames = 3;
  int ssd_person_class_id = 15;
};

struct TrackingParams {
  int tracking_button_index = 0;
  int lost_threshold = 150;
  int verification_interval = 30;
  int detection_miss_threshold = 3;
  double iou_threshold = 0.3;
  double kcf_scale = 0.5;
};

struct ScanParams {
  bool enabled = true;
  double pan_velocity = 0.5;
  double tilt_velocity = 0.3;
  double pan_counts_per_revolution = 6600000.0;
  double tilt_counts_per_revolution = 6600000.0;
  int pan_min_counts = -1650000;
  int pan_max_counts = 1650000;
  int tilt_min_counts = -500000;
  int tilt_max_counts = 300000;
  int tilt_step_counts = 200000;
};

struct AutoFireParams {
  bool enabled = false;
  double threshold = 0.7;
  double center_tolerance = 0.15;
  int cooldown_ms = 2000;
  int spinup_ms = 500;
  int fire_duration_ms = 300;
  double shooter_speed_rpm = 10000.0;

  double getShooterSpeedRadS() const {
    return shooter_speed_rpm * 2.0 * M_PI / 60.0;
  }
};

struct CameraParams {
  bool use_device = true;
  int device_id = 0;
  std::string image_topic = "/camera/image_raw";
  bool show_window = true;
};

}  // namespace body_tracker

#endif  // CHIKURIN_CONTROL_BODY_TRACKER_PARAMS_HPP
