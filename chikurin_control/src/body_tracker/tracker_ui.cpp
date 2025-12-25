#include "chikurin_control/body_tracker/tracker_ui.hpp"
#include <cstdio>
#include <algorithm>
#include <cmath>

namespace {
template<typename T>
T clamp(T val, T lo, T hi) {
  return std::max(lo, std::min(val, hi));
}
}

namespace body_tracker {

TrackerUI::TrackerUI(const PanTiltParams& params)
  : params_(params)
{
}

void TrackerUI::draw(cv::Mat& frame, const UIState& state)
{
  drawCrosshair(frame);
  drawStatusPanel(frame, state);
  drawButtonHint(frame, state.tracking_enabled);
  drawPanErrorBar(frame, state.pan_error);
  drawTiltGauge(frame, state.tilt_error);
  drawTarget(frame, state);
}

void TrackerUI::show(const cv::Mat& frame, const std::string& window_name)
{
  cv::imshow(window_name, frame);
  cv::waitKey(1);
}

void TrackerUI::drawCrosshair(cv::Mat& frame)
{
  int w = frame.cols;
  int h = frame.rows;

  cv::line(frame, cv::Point(w / 2, 0), cv::Point(w / 2, h),
    cv::Scalar(100, 100, 100), 1);
  cv::line(frame, cv::Point(0, h / 2), cv::Point(w, h / 2),
    cv::Scalar(100, 100, 100), 1);
}

void TrackerUI::drawStatusPanel(cv::Mat& frame, const UIState& state)
{
  // Status panel background
  cv::rectangle(frame, cv::Point(10, 10), cv::Point(280, 260),
    cv::Scalar(0, 0, 0), cv::FILLED);
  cv::rectangle(frame, cv::Point(10, 10), cv::Point(280, 260),
    cv::Scalar(100, 100, 100), 1);

  cv::Scalar status_color;
  std::string status_text;
  std::string mode_text;

  if (!state.tracking_enabled) {
    status_color = cv::Scalar(128, 128, 128);
    status_text = "IDLE";
    mode_text = "Press [A] to start";
  } else if (state.tracking_state == TrackingState::TRACKING) {
    status_color = cv::Scalar(0, 165, 255);
    status_text = "TRACKING";
    mode_text = "KCF";
  } else if (state.target_valid) {
    status_color = cv::Scalar(0, 255, 0);
    status_text = "DETECTING";
    mode_text = "MobileNet-SSD";
  } else {
    status_color = cv::Scalar(0, 0, 255);
    status_text = "SEARCHING";
    mode_text = "MobileNet-SSD";
  }

  cv::putText(frame, status_text, cv::Point(20, 35),
    cv::FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2);
  cv::putText(frame, mode_text, cv::Point(140, 35),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

  char buf[64];
  if (state.tracking_state == TrackingState::TRACKING) {
    snprintf(buf, sizeof(buf), "Lost: %d/%d", state.tracking_lost_count, state.lost_threshold);
  } else {
    snprintf(buf, sizeof(buf), "Persons: %zu", state.num_detections);
  }
  cv::putText(frame, buf, cv::Point(20, 55),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

  // Pan control info
  cv::putText(frame, "--- PAN ---", cv::Point(20, 80),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

  snprintf(buf, sizeof(buf), "Err: %+.3f  P:%+.4f I:%+.4f",
           state.pan_error, state.pan_p_term, state.pan_i_term);
  cv::putText(frame, buf, cv::Point(20, 100),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 200, 200), 1);

  snprintf(buf, sizeof(buf), "Cur: %+.3f  Tgt: %+.3f rad",
           state.current_pan, state.target_pan);
  cv::putText(frame, buf, cv::Point(20, 118),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);

  // Tilt control info
  cv::putText(frame, "--- TILT ---", cv::Point(20, 140),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

  snprintf(buf, sizeof(buf), "Err: %+.3f  P:%+.4f I:%+.4f",
           state.tilt_error, state.tilt_p_term, state.tilt_i_term);
  cv::putText(frame, buf, cv::Point(20, 160),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 255), 1);

  snprintf(buf, sizeof(buf), "Cur: %+.3f  Tgt: %+.3f rad",
           state.current_tilt, state.target_tilt);
  cv::putText(frame, buf, cv::Point(20, 178),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);

  // Base control info
  cv::putText(frame, "--- BASE ---", cv::Point(20, 200),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

  double pan_ratio = std::abs(state.current_pan) / params_.pan_limit;
  snprintf(buf, sizeof(buf), "Pan ratio: %.1f%% (thresh: %.0f%%)",
           pan_ratio * 100, params_.pan_base_threshold * 100);
  cv::putText(frame, buf, cv::Point(20, 218),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);

  snprintf(buf, sizeof(buf), "Angular vel: %+.3f rad/s", state.base_angular_velocity);
  cv::Scalar vel_color = std::abs(state.base_angular_velocity) > 0.01 ?
    cv::Scalar(0, 255, 255) : cv::Scalar(200, 200, 200);
  cv::putText(frame, buf, cv::Point(20, 236),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, vel_color, 1);

  // Gains
  snprintf(buf, sizeof(buf), "Pan Kp=%.4f Ki=%.5f", params_.pan_kp, params_.pan_ki);
  cv::putText(frame, buf, cv::Point(20, 253),
    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(150, 150, 150), 1);
}

void TrackerUI::drawButtonHint(cv::Mat& frame, bool tracking_enabled)
{
  int w = frame.cols;
  std::string hint = tracking_enabled ? "[A] STOP" : "[A] START";
  cv::Scalar hint_color = tracking_enabled ? cv::Scalar(0, 100, 255) : cv::Scalar(0, 255, 0);
  cv::putText(frame, hint, cv::Point(w - 100, 25),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, hint_color, 2);
}

void TrackerUI::drawPanErrorBar(cv::Mat& frame, double pan_error)
{
  int w = frame.cols;
  int h = frame.rows;
  int bar_y = h - 40;
  int bar_width = w - 80;
  int bar_height = 15;
  int bar_x = 40;

  cv::putText(frame, "PAN", cv::Point(10, bar_y + 12),
    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(200, 200, 200), 1);

  cv::rectangle(frame, cv::Point(bar_x, bar_y),
    cv::Point(bar_x + bar_width, bar_y + bar_height),
    cv::Scalar(50, 50, 50), cv::FILLED);

  cv::line(frame, cv::Point(bar_x + bar_width / 2, bar_y),
    cv::Point(bar_x + bar_width / 2, bar_y + bar_height),
    cv::Scalar(255, 255, 255), 2);

  int pan_pos = bar_x + bar_width / 2 - static_cast<int>(pan_error * bar_width / 2);
  pan_pos = clamp(pan_pos, bar_x, bar_x + bar_width);
  cv::circle(frame, cv::Point(pan_pos, bar_y + bar_height / 2),
    6, cv::Scalar(0, 255, 255), cv::FILLED);
}

void TrackerUI::drawTiltGauge(cv::Mat& frame, double tilt_error)
{
  int w = frame.cols;
  int h = frame.rows;
  int gauge_x = w - 50;
  int gauge_y = 80;
  int gauge_height = h - 160;
  int gauge_width = 25;

  cv::putText(frame, "TILT", cv::Point(gauge_x - 5, gauge_y - 10),
    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(200, 200, 200), 1);

  cv::rectangle(frame, cv::Point(gauge_x, gauge_y),
    cv::Point(gauge_x + gauge_width, gauge_y + gauge_height),
    cv::Scalar(50, 50, 50), cv::FILLED);

  cv::line(frame, cv::Point(gauge_x, gauge_y + gauge_height / 2),
    cv::Point(gauge_x + gauge_width, gauge_y + gauge_height / 2),
    cv::Scalar(255, 255, 255), 1);

  int tilt_pos = gauge_y + gauge_height / 2 - static_cast<int>(tilt_error * gauge_height / 2);
  tilt_pos = clamp(tilt_pos, gauge_y, gauge_y + gauge_height);
  cv::circle(frame, cv::Point(gauge_x + gauge_width / 2, tilt_pos),
    6, cv::Scalar(0, 255, 255), cv::FILLED);
}

void TrackerUI::drawTarget(cv::Mat& frame, const UIState& state)
{
  if (!state.target_valid) {
    return;
  }

  int body_center_x = state.target_box.x + state.target_box.width / 2;
  int body_center_y = state.target_box.y + state.target_box.height / 2;
  int frame_center_x = frame.cols / 2;
  int frame_center_y = frame.rows / 2;

  if (state.tracking_state == TrackingState::TRACKING) {
    cv::rectangle(frame, state.target_box, cv::Scalar(0, 165, 255), 3);
  } else {
    cv::rectangle(frame, state.target_box, cv::Scalar(0, 255, 0), 2);
    if (state.target_confidence > 0) {
      char conf_buf[32];
      snprintf(conf_buf, sizeof(conf_buf), "%.1f%%", state.target_confidence * 100);
      cv::putText(frame, conf_buf, cv::Point(state.target_box.x, state.target_box.y - 5),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
  }

  cv::drawMarker(frame, cv::Point(body_center_x, body_center_y),
    cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
  cv::line(frame, cv::Point(frame_center_x, frame_center_y),
    cv::Point(body_center_x, body_center_y), cv::Scalar(255, 255, 0), 2);
}

}  // namespace body_tracker
