#ifndef CHIKURIN_CONTROL_BODY_TRACKER_PERSON_DETECTOR_HPP
#define CHIKURIN_CONTROL_BODY_TRACKER_PERSON_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "chikurin_control/body_tracker/params.hpp"

namespace body_tracker {

struct DetectionResult {
  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> indices;

  bool empty() const { return indices.empty(); }
  size_t count() const { return indices.size(); }
};

class PersonDetector {
public:
  explicit PersonDetector(const DetectionParams& params);
  ~PersonDetector();

  PersonDetector(const PersonDetector&) = delete;
  PersonDetector& operator=(const PersonDetector&) = delete;

  bool loadModel(const std::string& prototxt_path, const std::string& caffemodel_path);

  DetectionResult detect(const cv::Mat& frame);

  void requestAsyncDetection(const cv::Mat& frame);
  bool getAsyncResult(DetectionResult& result);
  bool isAsyncPending() const { return request_pending_; }

  cv::Rect selectBestTarget(const DetectionResult& result, int frame_width, int frame_height) const;

  static double calculateIoU(const cv::Rect& a, const cv::Rect& b);

private:
  void asyncThreadFunc();

  DetectionParams params_;
  cv::dnn::Net net_;

  std::thread async_thread_;
  std::atomic<bool> thread_running_{false};
  std::mutex request_mutex_;
  std::condition_variable request_cv_;
  cv::Mat pending_frame_;
  std::atomic<bool> request_pending_{false};

  std::mutex result_mutex_;
  DetectionResult async_result_;
  std::atomic<bool> result_ready_{false};
};

}  // namespace body_tracker

#endif  // CHIKURIN_CONTROL_BODY_TRACKER_PERSON_DETECTOR_HPP
