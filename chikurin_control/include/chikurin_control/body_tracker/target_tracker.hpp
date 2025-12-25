#ifndef CHIKURIN_CONTROL_BODY_TRACKER_TARGET_TRACKER_HPP
#define CHIKURIN_CONTROL_BODY_TRACKER_TARGET_TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include "chikurin_control/body_tracker/params.hpp"

namespace body_tracker {

enum class TrackingState {
  DETECTING,
  TRACKING
};

class TargetTracker {
public:
  explicit TargetTracker(const TrackingParams& params);

  bool init(const cv::Mat& frame, const cv::Rect& bbox);
  bool update(const cv::Mat& frame);
  void reset();

  bool reinit(const cv::Mat& frame, const cv::Rect& bbox);

  TrackingState getState() const { return state_; }
  void setState(TrackingState state) { state_ = state; }

  cv::Rect getBoundingBox() const { return tracked_bbox_; }
  bool isInitialized() const { return initialized_; }

  int getLostCount() const { return lost_count_; }
  void incrementLostCount() { lost_count_++; }
  void resetLostCount() { lost_count_ = 0; }
  bool isLost() const { return lost_count_ >= params_.lost_threshold; }

  int getVerificationCounter() const { return verification_counter_; }
  void incrementVerificationCounter() { verification_counter_++; }
  void resetVerificationCounter() { verification_counter_ = 0; }
  bool needsVerification() const {
    return verification_counter_ >= params_.verification_interval;
  }

  int getDetectionMissCount() const { return detection_miss_count_; }
  void incrementDetectionMissCount() { detection_miss_count_++; }
  void resetDetectionMissCount() { detection_miss_count_ = 0; }
  bool hasExceededMissThreshold() const {
    return detection_miss_count_ >= params_.detection_miss_threshold;
  }

  double getIoUThreshold() const { return params_.iou_threshold; }

private:
  cv::Ptr<cv::TrackerKCF> createKCFTracker();
  cv::Mat resizeForKCF(const cv::Mat& frame);
  cv::Rect scaleBBoxForKCF(const cv::Rect& bbox);
  cv::Rect scaleBBoxFromKCF(const cv::Rect& bbox);

  TrackingParams params_;
  cv::Ptr<cv::Tracker> tracker_;
  cv::Rect tracked_bbox_;
  bool initialized_ = false;
  TrackingState state_ = TrackingState::DETECTING;

  int lost_count_ = 0;
  int verification_counter_ = 0;
  int detection_miss_count_ = 0;
};

}  // namespace body_tracker

#endif  // CHIKURIN_CONTROL_BODY_TRACKER_TARGET_TRACKER_HPP
