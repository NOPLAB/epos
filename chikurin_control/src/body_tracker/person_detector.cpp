#include "chikurin_control/body_tracker/person_detector.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

namespace body_tracker {

PersonDetector::PersonDetector(const DetectionParams& params)
  : params_(params)
{
  thread_running_ = true;
  async_thread_ = std::thread(&PersonDetector::asyncThreadFunc, this);
}

PersonDetector::~PersonDetector()
{
  thread_running_ = false;
  request_cv_.notify_all();
  if (async_thread_.joinable()) {
    async_thread_.join();
  }
}

bool PersonDetector::loadModel(const std::string& prototxt_path, const std::string& caffemodel_path)
{
  try {
    net_ = cv::dnn::readNetFromCaffe(prototxt_path, caffemodel_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    return true;
  } catch (const cv::Exception& e) {
    return false;
  }
}

DetectionResult PersonDetector::detect(const cv::Mat& frame)
{
  DetectionResult result;
  int width = frame.cols;
  int height = frame.rows;

  cv::Mat blob;
  cv::dnn::blobFromImage(frame, blob, 0.007843, cv::Size(300, 300),
                         cv::Scalar(127.5, 127.5, 127.5), false, false);
  net_.setInput(blob);

  cv::Mat detection = net_.forward();
  cv::Mat detections(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

  for (int i = 0; i < detections.rows; i++) {
    float confidence = detections.at<float>(i, 2);
    int class_id = static_cast<int>(detections.at<float>(i, 1));

    if (class_id == params_.ssd_person_class_id && confidence > params_.confidence_threshold) {
      int x1 = static_cast<int>(detections.at<float>(i, 3) * width);
      int y1 = static_cast<int>(detections.at<float>(i, 4) * height);
      int x2 = static_cast<int>(detections.at<float>(i, 5) * width);
      int y2 = static_cast<int>(detections.at<float>(i, 6) * height);

      x1 = std::max(0, std::min(x1, width - 1));
      y1 = std::max(0, std::min(y1, height - 1));
      x2 = std::max(0, std::min(x2, width - 1));
      y2 = std::max(0, std::min(y2, height - 1));

      int w = x2 - x1;
      int h = y2 - y1;

      if (w > 0 && h > 0) {
        result.boxes.push_back(cv::Rect(x1, y1, w, h));
        result.confidences.push_back(confidence);
      }
    }
  }

  cv::dnn::NMSBoxes(result.boxes, result.confidences,
                    static_cast<float>(params_.confidence_threshold),
                    static_cast<float>(params_.nms_threshold),
                    result.indices);

  return result;
}

void PersonDetector::requestAsyncDetection(const cv::Mat& frame)
{
  std::lock_guard<std::mutex> lock(request_mutex_);
  pending_frame_ = frame.clone();
  request_pending_ = true;
  request_cv_.notify_one();
}

bool PersonDetector::getAsyncResult(DetectionResult& result)
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  if (result_ready_) {
    result = async_result_;
    result_ready_ = false;
    return true;
  }
  return false;
}

cv::Rect PersonDetector::selectBestTarget(const DetectionResult& result,
                                          int frame_width, int frame_height) const
{
  int center_x = frame_width / 2;
  int center_y = frame_height / 2;

  cv::Rect best_box;
  double min_dist = std::numeric_limits<double>::max();

  for (int idx : result.indices) {
    const cv::Rect& box = result.boxes[idx];
    int box_center_x = box.x + box.width / 2;
    int box_center_y = box.y + box.height / 2;

    double dist = std::hypot(box_center_x - center_x, box_center_y - center_y);
    if (dist < min_dist) {
      min_dist = dist;
      best_box = box;
    }
  }

  return best_box;
}

double PersonDetector::calculateIoU(const cv::Rect& a, const cv::Rect& b)
{
  int x1 = std::max(a.x, b.x);
  int y1 = std::max(a.y, b.y);
  int x2 = std::min(a.x + a.width, b.x + b.width);
  int y2 = std::min(a.y + a.height, b.y + b.height);

  if (x2 <= x1 || y2 <= y1) {
    return 0.0;
  }

  double intersection = (x2 - x1) * (y2 - y1);
  double union_area = a.area() + b.area() - intersection;

  return intersection / union_area;
}

void PersonDetector::asyncThreadFunc()
{
  while (thread_running_) {
    cv::Mat frame_to_process;

    {
      std::unique_lock<std::mutex> lock(request_mutex_);
      request_cv_.wait(lock, [this] {
        return !thread_running_ || request_pending_;
      });

      if (!thread_running_) {
        break;
      }

      frame_to_process = pending_frame_.clone();
      request_pending_ = false;
    }

    DetectionResult result = detect(frame_to_process);

    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      async_result_ = result;
      result_ready_ = true;
    }
  }
}

}  // namespace body_tracker
