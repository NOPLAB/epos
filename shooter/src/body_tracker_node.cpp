#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

class BodyTrackerNode : public rclcpp::Node
{
public:
  BodyTrackerNode() : Node("body_tracker")
  {
    // Parameters
    declare_parameter("kp", 0.0);
    declare_parameter("ki", 0.0);
    declare_parameter("max_angular_velocity", 1.5);
    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("use_camera_device", true);
    declare_parameter("camera_device_id", 0);
    declare_parameter("confidence_threshold", 0.5);
    declare_parameter("nms_threshold", 0.4);

    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
    use_camera_device_ = get_parameter("use_camera_device").as_bool();
    camera_device_id_ = get_parameter("camera_device_id").as_int();
    confidence_threshold_ = get_parameter("confidence_threshold").as_double();
    nms_threshold_ = get_parameter("nms_threshold").as_double();

    // Load YOLO model
    std::string pkg_share = ament_index_cpp::get_package_share_directory("shooter");
    std::string cfg_path = pkg_share + "/models/yolov4-tiny.cfg";
    std::string weights_path = pkg_share + "/models/yolov4-tiny.weights";
    std::string names_path = pkg_share + "/models/coco.names";

    RCLCPP_INFO(get_logger(), "Loading YOLO model from: %s", pkg_share.c_str());

    // Load class names
    std::ifstream ifs(names_path);
    std::string line;
    while (std::getline(ifs, line)) {
      class_names_.push_back(line);
    }

    // Load network
    net_ = cv::dnn::readNetFromDarknet(cfg_path, weights_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // Get output layer names
    output_layers_ = net_.getUnconnectedOutLayersNames();

    RCLCPP_INFO(get_logger(), "YOLO model loaded successfully (%zu classes)", class_names_.size());

    // Publisher
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive_controller/cmd_vel_unstamped", 10);

    if (use_camera_device_) {
      // Use OpenCV VideoCapture directly
      cap_.open(camera_device_id_);
      if (!cap_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera device %d", camera_device_id_);
        throw std::runtime_error("Failed to open camera");
      }
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

      // Timer for camera capture
      timer_ = create_wall_timer(
        std::chrono::milliseconds(33),  // ~30 FPS
        std::bind(&BodyTrackerNode::camera_callback, this));
    } else {
      // Subscribe to ROS image topic
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("image_topic").as_string(), 10,
        std::bind(&BodyTrackerNode::image_callback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(), "Body tracker node started (Kp=%.4f, Ki=%.4f)", kp_, ki_);
  }

  ~BodyTrackerNode()
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void camera_callback()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN(get_logger(), "Failed to read frame from camera");
      return;
    }
    process_frame(frame);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    process_frame(cv_ptr->image);
  }

  void process_frame(cv::Mat& frame)
  {
    int width = frame.cols;
    int height = frame.rows;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(416, 416),
                           cv::Scalar(0, 0, 0), true, false);
    net_.setInput(blob);

    // Forward pass
    std::vector<cv::Mat> outs;
    net_.forward(outs, output_layers_);

    // Parse detections
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (const auto& out : outs) {
      auto* data = (float*)out.data;
      for (int i = 0; i < out.rows; ++i, data += out.cols) {
        cv::Mat scores = out.row(i).colRange(5, out.cols);
        cv::Point class_id_point;
        double confidence;
        cv::minMaxLoc(scores, nullptr, &confidence, nullptr, &class_id_point);

        // Only detect "person" (class 0) with sufficient confidence
        if (class_id_point.x == 0 && confidence > confidence_threshold_) {
          int center_x = static_cast<int>(data[0] * width);
          int center_y = static_cast<int>(data[1] * height);
          int w = static_cast<int>(data[2] * width);
          int h = static_cast<int>(data[3] * height);
          int x = center_x - w / 2;
          int y = center_y - h / 2;

          boxes.push_back(cv::Rect(x, y, w, h));
          confidences.push_back(static_cast<float>(confidence));
          class_ids.push_back(class_id_point.x);
        }
      }
    }

    // Non-maximum suppression
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, indices);

    double angular_velocity = 0.0;
    double error = 0.0;
    double p_term = 0.0;
    double i_term = 0.0;
    int frame_center_x = width / 2;
    bool body_detected = false;
    float best_confidence = 0.0;
    cv::Rect best_box;

    // Find the largest/most confident person
    for (int idx : indices) {
      if (confidences[idx] > best_confidence) {
        best_confidence = confidences[idx];
        best_box = boxes[idx];
        body_detected = true;
      }
    }

    if (body_detected) {
      // Calculate body center
      int body_center_x = best_box.x + best_box.width / 2;
      int body_center_y = best_box.y + best_box.height / 2;

      // Calculate error (normalized to -1.0 ~ 1.0)
      error = static_cast<double>(frame_center_x - body_center_x) / frame_center_x;

      // PI control
      integral_error_ += error;

      // Anti-windup
      double max_integral = max_angular_velocity_ / ki_;
      integral_error_ = std::clamp(integral_error_, -max_integral, max_integral);

      p_term = kp_ * error;
      i_term = ki_ * integral_error_;
      angular_velocity = p_term + i_term;
      angular_velocity = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);

      // Draw body rectangle
      cv::rectangle(frame, best_box, cv::Scalar(0, 255, 0), 2);

      // Draw confidence
      char conf_buf[32];
      snprintf(conf_buf, sizeof(conf_buf), "%.1f%%", best_confidence * 100);
      cv::putText(frame, conf_buf, cv::Point(best_box.x, best_box.y - 5),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

      // Draw body center crosshair
      cv::drawMarker(frame, cv::Point(body_center_x, body_center_y),
        cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);

      // Draw line from center to body
      cv::line(frame, cv::Point(frame_center_x, height / 2),
        cv::Point(body_center_x, body_center_y), cv::Scalar(255, 255, 0), 2);

      // Draw all detected persons (faded)
      for (int idx : indices) {
        if (boxes[idx] != best_box) {
          cv::rectangle(frame, boxes[idx], cv::Scalar(100, 100, 100), 1);
        }
      }
    } else {
      // No body detected - slowly reset integral
      integral_error_ *= 0.95;
    }

    // Publish velocity command
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = angular_velocity;
    cmd_vel_pub_->publish(msg);

    // === Draw UI ===
    draw_ui(frame, body_detected, error, p_term, i_term, angular_velocity, indices.size());

    cv::imshow("Body Tracker", frame);
    cv::waitKey(1);
  }

  void draw_ui(cv::Mat& frame, bool body_detected, double error,
               double p_term, double i_term, double angular_velocity, size_t num_detections)
  {
    int w = frame.cols;
    int h = frame.rows;

    // Draw center line
    cv::line(frame, cv::Point(w / 2, 0), cv::Point(w / 2, h),
      cv::Scalar(100, 100, 100), 1);

    // Status panel background
    cv::rectangle(frame, cv::Point(10, 10), cv::Point(250, 180),
      cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(frame, cv::Point(10, 10), cv::Point(250, 180),
      cv::Scalar(100, 100, 100), 1);

    // Status text
    cv::Scalar status_color = body_detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    std::string status_text = body_detected ? "TRACKING" : "SEARCHING";
    cv::putText(frame, status_text, cv::Point(20, 35),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2);

    // Detection method
    cv::putText(frame, "YOLO v4-tiny", cv::Point(140, 35),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

    // Detection count
    char buf[64];
    snprintf(buf, sizeof(buf), "Persons: %zu", num_detections);
    cv::putText(frame, buf, cv::Point(20, 55),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

    // PI values
    snprintf(buf, sizeof(buf), "Error: %+.3f", error);
    cv::putText(frame, buf, cv::Point(20, 80),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    snprintf(buf, sizeof(buf), "P: %+.4f", p_term);
    cv::putText(frame, buf, cv::Point(20, 100),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 200, 200), 1);

    snprintf(buf, sizeof(buf), "I: %+.4f", i_term);
    cv::putText(frame, buf, cv::Point(20, 120),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 255), 1);

    snprintf(buf, sizeof(buf), "Cmd: %+.3f rad/s", angular_velocity);
    cv::putText(frame, buf, cv::Point(20, 140),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

    // Gains
    snprintf(buf, sizeof(buf), "Kp=%.4f Ki=%.5f", kp_, ki_);
    cv::putText(frame, buf, cv::Point(20, 170),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

    // Error bar (bottom of screen)
    int bar_y = h - 40;
    int bar_width = w - 40;
    int bar_height = 20;
    int bar_x = 20;

    // Bar background
    cv::rectangle(frame, cv::Point(bar_x, bar_y),
      cv::Point(bar_x + bar_width, bar_y + bar_height),
      cv::Scalar(50, 50, 50), cv::FILLED);

    // Center marker
    cv::line(frame, cv::Point(bar_x + bar_width / 2, bar_y),
      cv::Point(bar_x + bar_width / 2, bar_y + bar_height),
      cv::Scalar(255, 255, 255), 2);

    // Error indicator
    int error_pos = bar_x + bar_width / 2 - static_cast<int>(error * bar_width / 2);
    error_pos = std::clamp(error_pos, bar_x, bar_x + bar_width);
    cv::circle(frame, cv::Point(error_pos, bar_y + bar_height / 2),
      8, cv::Scalar(0, 255, 255), cv::FILLED);

    // Velocity gauge (right side)
    int gauge_x = w - 50;
    int gauge_y = 80;
    int gauge_height = h - 160;
    int gauge_width = 30;

    // Gauge background
    cv::rectangle(frame, cv::Point(gauge_x, gauge_y),
      cv::Point(gauge_x + gauge_width, gauge_y + gauge_height),
      cv::Scalar(50, 50, 50), cv::FILLED);

    // Center line
    cv::line(frame, cv::Point(gauge_x, gauge_y + gauge_height / 2),
      cv::Point(gauge_x + gauge_width, gauge_y + gauge_height / 2),
      cv::Scalar(255, 255, 255), 1);

    // Velocity bar
    double vel_ratio = angular_velocity / max_angular_velocity_;
    int vel_bar_height = static_cast<int>(std::abs(vel_ratio) * gauge_height / 2);
    int vel_bar_y = gauge_y + gauge_height / 2;
    cv::Scalar vel_color = (angular_velocity >= 0) ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 200);

    if (angular_velocity >= 0) {
      cv::rectangle(frame, cv::Point(gauge_x + 2, vel_bar_y - vel_bar_height),
        cv::Point(gauge_x + gauge_width - 2, vel_bar_y), vel_color, cv::FILLED);
    } else {
      cv::rectangle(frame, cv::Point(gauge_x + 2, vel_bar_y),
        cv::Point(gauge_x + gauge_width - 2, vel_bar_y + vel_bar_height), vel_color, cv::FILLED);
    }

    // Gauge label
    cv::putText(frame, "VEL", cv::Point(gauge_x, gauge_y - 10),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
  }

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // OpenCV / YOLO
  cv::dnn::Net net_;
  std::vector<std::string> output_layers_;
  std::vector<std::string> class_names_;
  cv::VideoCapture cap_;

  // Parameters
  double kp_;
  double ki_;
  double max_angular_velocity_;
  bool use_camera_device_;
  int camera_device_id_;
  double confidence_threshold_;
  double nms_threshold_;

  // PI control state
  double integral_error_ = 0.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BodyTrackerNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("body_tracker"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
