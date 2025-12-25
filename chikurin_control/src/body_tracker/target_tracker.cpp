#include "chikurin_control/body_tracker/target_tracker.hpp"

namespace body_tracker {

TargetTracker::TargetTracker(const TrackingParams& params)
  : params_(params)
{
}

cv::Ptr<cv::TrackerKCF> TargetTracker::createKCFTracker()
{
  cv::TrackerKCF::Params kcf_params;
  kcf_params.detect_thresh = 0.4;
  kcf_params.resize = true;
  return cv::TrackerKCF::create(kcf_params);
}

cv::Mat TargetTracker::resizeForKCF(const cv::Mat& frame)
{
  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(), params_.kcf_scale, params_.kcf_scale, cv::INTER_LINEAR);
  return resized;
}

cv::Rect TargetTracker::scaleBBoxForKCF(const cv::Rect& bbox)
{
  return cv::Rect(
    static_cast<int>(bbox.x * params_.kcf_scale),
    static_cast<int>(bbox.y * params_.kcf_scale),
    static_cast<int>(bbox.width * params_.kcf_scale),
    static_cast<int>(bbox.height * params_.kcf_scale)
  );
}

cv::Rect TargetTracker::scaleBBoxFromKCF(const cv::Rect& bbox)
{
  return cv::Rect(
    static_cast<int>(bbox.x / params_.kcf_scale),
    static_cast<int>(bbox.y / params_.kcf_scale),
    static_cast<int>(bbox.width / params_.kcf_scale),
    static_cast<int>(bbox.height / params_.kcf_scale)
  );
}

bool TargetTracker::init(const cv::Mat& frame, const cv::Rect& bbox)
{
  try {
    tracker_ = createKCFTracker();

    cv::Mat kcf_frame = resizeForKCF(frame);
    cv::Rect kcf_bbox = scaleBBoxForKCF(bbox);
    tracker_->init(kcf_frame, kcf_bbox);

    tracked_bbox_ = bbox;
    initialized_ = true;
    state_ = TrackingState::TRACKING;
    lost_count_ = 0;
    verification_counter_ = 0;
    detection_miss_count_ = 0;

    return true;
  } catch (const cv::Exception&) {
    return false;
  }
}

bool TargetTracker::update(const cv::Mat& frame)
{
  if (!initialized_ || tracker_.empty()) {
    return false;
  }

  cv::Mat kcf_frame = resizeForKCF(frame);
  cv::Rect kcf_bbox = scaleBBoxForKCF(tracked_bbox_);

  bool success = tracker_->update(kcf_frame, kcf_bbox);

  if (success) {
    tracked_bbox_ = scaleBBoxFromKCF(kcf_bbox);
  }

  return success;
}

bool TargetTracker::reinit(const cv::Mat& frame, const cv::Rect& bbox)
{
  try {
    tracker_ = createKCFTracker();
    cv::Mat kcf_frame = resizeForKCF(frame);
    cv::Rect kcf_bbox = scaleBBoxForKCF(bbox);
    tracker_->init(kcf_frame, kcf_bbox);
    tracked_bbox_ = bbox;
    return true;
  } catch (const cv::Exception&) {
    return false;
  }
}

void TargetTracker::reset()
{
  tracker_.release();
  initialized_ = false;
  state_ = TrackingState::DETECTING;
  lost_count_ = 0;
  verification_counter_ = 0;
  detection_miss_count_ = 0;
  tracked_bbox_ = cv::Rect();
}

}  // namespace body_tracker
