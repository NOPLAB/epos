#ifndef CHIKURIN_CONTROL_BODY_TRACKER_TRACKER_UI_HPP
#define CHIKURIN_CONTROL_BODY_TRACKER_TRACKER_UI_HPP

#include <opencv2/opencv.hpp>
#include <string>

#include "chikurin_control/body_tracker/params.hpp"
#include "chikurin_control/body_tracker/target_tracker.hpp"
#include "chikurin_control/body_tracker/pan_tilt_controller.hpp"

namespace body_tracker {

struct UIState {
  bool tracking_enabled = false;
  TrackingState tracking_state = TrackingState::DETECTING;
  bool target_valid = false;
  cv::Rect target_box;
  float target_confidence = 0.0f;

  double pan_error = 0.0;
  double tilt_error = 0.0;
  double pan_p_term = 0.0;
  double pan_i_term = 0.0;
  double tilt_p_term = 0.0;
  double tilt_i_term = 0.0;

  double current_pan = 0.0;
  double current_tilt = 0.0;
  double target_pan = 0.0;
  double target_tilt = 0.0;

  double base_angular_velocity = 0.0;
  size_t num_detections = 0;
  int tracking_lost_count = 0;
  int lost_threshold = 150;
};

class TrackerUI {
public:
  explicit TrackerUI(const PanTiltParams& params);

  void draw(cv::Mat& frame, const UIState& state);
  void show(const cv::Mat& frame, const std::string& window_name = "Body Tracker");

private:
  void drawCrosshair(cv::Mat& frame);
  void drawStatusPanel(cv::Mat& frame, const UIState& state);
  void drawButtonHint(cv::Mat& frame, bool tracking_enabled);
  void drawPanErrorBar(cv::Mat& frame, double pan_error);
  void drawTiltGauge(cv::Mat& frame, double tilt_error);
  void drawTarget(cv::Mat& frame, const UIState& state);

  PanTiltParams params_;
};

}  // namespace body_tracker

#endif  // CHIKURIN_CONTROL_BODY_TRACKER_TRACKER_UI_HPP
