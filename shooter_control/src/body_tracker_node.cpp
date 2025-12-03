#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <cmath>

class BodyTrackerNode : public rclcpp::Node
{
public:
  BodyTrackerNode() : Node("body_tracker")
  {
    // Parameters
    declare_parameter("pan_kp", 0.002);
    declare_parameter("pan_ki", 0.0001);
    declare_parameter("tilt_kp", 0.002);
    declare_parameter("tilt_ki", 0.0001);
    declare_parameter("pan_limit", M_PI);        // ±π rad
    declare_parameter("tilt_limit", M_PI / 2);   // ±π/2 rad
    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("use_camera_device", true);
    declare_parameter("camera_device_id", 0);
    declare_parameter("confidence_threshold", 0.5);
    declare_parameter("nms_threshold", 0.4);

    pan_kp_ = get_parameter("pan_kp").as_double();
    pan_ki_ = get_parameter("pan_ki").as_double();
    tilt_kp_ = get_parameter("tilt_kp").as_double();
    tilt_ki_ = get_parameter("tilt_ki").as_double();
    pan_limit_ = get_parameter("pan_limit").as_double();
    tilt_limit_ = get_parameter("tilt_limit").as_double();
    use_camera_device_ = get_parameter("use_camera_device").as_bool();
    camera_device_id_ = get_parameter("camera_device_id").as_int();
    confidence_threshold_ = get_parameter("confidence_threshold").as_double();
    nms_threshold_ = get_parameter("nms_threshold").as_double();

    // Load YOLO model
    std::string pkg_share = ament_index_cpp::get_package_share_directory("shooter_control");
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

    // Publisher for pan-tilt position commands
    pan_tilt_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pan_tilt_controller/commands", 10);

    // Subscriber for current joint state
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&BodyTrackerNode::joint_state_callback, this, std::placeholders::_1));

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

    RCLCPP_INFO(get_logger(), "Body tracker node started (pan: Kp=%.4f Ki=%.5f, tilt: Kp=%.4f Ki=%.5f)",
                pan_kp_, pan_ki_, tilt_kp_, tilt_ki_);
  }

  ~BodyTrackerNode()
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "pan_joint" && i < msg->position.size()) {
        current_pan_ = msg->position[i];
      } else if (msg->name[i] == "tilt_joint" && i < msg->position.size()) {
        current_tilt_ = msg->position[i];
      }
    }
  }

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

    int frame_center_x = width / 2;
    int frame_center_y = height / 2;
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

    double pan_error = 0.0;
    double tilt_error = 0.0;
    double pan_p_term = 0.0, pan_i_term = 0.0;
    double tilt_p_term = 0.0, tilt_i_term = 0.0;
    double target_pan = current_pan_;
    double target_tilt = current_tilt_;

    if (body_detected) {
      // Calculate body center
      int body_center_x = best_box.x + best_box.width / 2;
      int body_center_y = best_box.y + best_box.height / 2;

      // Calculate errors (normalized to -1.0 ~ 1.0)
      // Pan: positive error means body is to the left, need to rotate left (positive pan)
      pan_error = static_cast<double>(frame_center_x - body_center_x) / frame_center_x;
      // Tilt: positive error means body is above center, need to tilt up (negative tilt)
      tilt_error = static_cast<double>(frame_center_y - body_center_y) / frame_center_y;

      // PI control for pan
      pan_integral_error_ += pan_error;
      double pan_max_integral = pan_limit_ / (pan_ki_ + 1e-6);
      pan_integral_error_ = std::clamp(pan_integral_error_, -pan_max_integral, pan_max_integral);
      pan_p_term = pan_kp_ * pan_error;
      pan_i_term = pan_ki_ * pan_integral_error_;
      double pan_delta = pan_p_term + pan_i_term;

      // PI control for tilt
      tilt_integral_error_ += tilt_error;
      double tilt_max_integral = tilt_limit_ / (tilt_ki_ + 1e-6);
      tilt_integral_error_ = std::clamp(tilt_integral_error_, -tilt_max_integral, tilt_max_integral);
      tilt_p_term = tilt_kp_ * tilt_error;
      tilt_i_term = tilt_ki_ * tilt_integral_error_;
      double tilt_delta = -(tilt_p_term + tilt_i_term);  // Negative because tilt axis is inverted

      // Calculate target positions
      target_pan = current_pan_ + pan_delta;
      target_tilt = current_tilt_ + tilt_delta;

      // Clamp to limits
      target_pan = std::clamp(target_pan, -pan_limit_, pan_limit_);
      target_tilt = std::clamp(target_tilt, -tilt_limit_, tilt_limit_);

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
      cv::line(frame, cv::Point(frame_center_x, frame_center_y),
        cv::Point(body_center_x, body_center_y), cv::Scalar(255, 255, 0), 2);

      // Draw all detected persons (faded)
      for (int idx : indices) {
        if (boxes[idx] != best_box) {
          cv::rectangle(frame, boxes[idx], cv::Scalar(100, 100, 100), 1);
        }
      }
    } else {
      // No body detected - slowly reset integral
      pan_integral_error_ *= 0.95;
      tilt_integral_error_ *= 0.95;
    }

    // Publish pan-tilt position command
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(2);
    msg.data[0] = target_pan;
    msg.data[1] = target_tilt;
    pan_tilt_pub_->publish(msg);

    // === Draw UI ===
    draw_ui(frame, body_detected, pan_error, tilt_error,
            pan_p_term, pan_i_term, tilt_p_term, tilt_i_term,
            target_pan, target_tilt, indices.size());

    cv::imshow("Body Tracker", frame);
    cv::waitKey(1);
  }

  void draw_ui(cv::Mat& frame, bool body_detected,
               double pan_error, double tilt_error,
               double pan_p, double pan_i, double tilt_p, double tilt_i,
               double target_pan, double target_tilt, size_t num_detections)
  {
    int w = frame.cols;
    int h = frame.rows;

    // Draw crosshair at center
    cv::line(frame, cv::Point(w / 2, 0), cv::Point(w / 2, h),
      cv::Scalar(100, 100, 100), 1);
    cv::line(frame, cv::Point(0, h / 2), cv::Point(w, h / 2),
      cv::Scalar(100, 100, 100), 1);

    // Status panel background
    cv::rectangle(frame, cv::Point(10, 10), cv::Point(280, 220),
      cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(frame, cv::Point(10, 10), cv::Point(280, 220),
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

    // Pan control info
    cv::putText(frame, "--- PAN ---", cv::Point(20, 80),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

    snprintf(buf, sizeof(buf), "Err: %+.3f  P:%+.4f I:%+.4f", pan_error, pan_p, pan_i);
    cv::putText(frame, buf, cv::Point(20, 100),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 200, 200), 1);

    snprintf(buf, sizeof(buf), "Cur: %+.3f  Tgt: %+.3f rad", current_pan_, target_pan);
    cv::putText(frame, buf, cv::Point(20, 118),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);

    // Tilt control info
    cv::putText(frame, "--- TILT ---", cv::Point(20, 140),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

    snprintf(buf, sizeof(buf), "Err: %+.3f  P:%+.4f I:%+.4f", tilt_error, tilt_p, tilt_i);
    cv::putText(frame, buf, cv::Point(20, 160),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 255), 1);

    snprintf(buf, sizeof(buf), "Cur: %+.3f  Tgt: %+.3f rad", current_tilt_, target_tilt);
    cv::putText(frame, buf, cv::Point(20, 178),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);

    // Gains
    snprintf(buf, sizeof(buf), "Pan Kp=%.4f Ki=%.5f", pan_kp_, pan_ki_);
    cv::putText(frame, buf, cv::Point(20, 200),
      cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(150, 150, 150), 1);
    snprintf(buf, sizeof(buf), "Tilt Kp=%.4f Ki=%.5f", tilt_kp_, tilt_ki_);
    cv::putText(frame, buf, cv::Point(20, 215),
      cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(150, 150, 150), 1);

    // Pan error bar (bottom of screen)
    int bar_y = h - 40;
    int bar_width = w - 80;
    int bar_height = 15;
    int bar_x = 40;

    cv::putText(frame, "PAN", cv::Point(10, bar_y + 12),
      cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(200, 200, 200), 1);

    // Bar background
    cv::rectangle(frame, cv::Point(bar_x, bar_y),
      cv::Point(bar_x + bar_width, bar_y + bar_height),
      cv::Scalar(50, 50, 50), cv::FILLED);

    // Center marker
    cv::line(frame, cv::Point(bar_x + bar_width / 2, bar_y),
      cv::Point(bar_x + bar_width / 2, bar_y + bar_height),
      cv::Scalar(255, 255, 255), 2);

    // Pan error indicator
    int pan_pos = bar_x + bar_width / 2 - static_cast<int>(pan_error * bar_width / 2);
    pan_pos = std::clamp(pan_pos, bar_x, bar_x + bar_width);
    cv::circle(frame, cv::Point(pan_pos, bar_y + bar_height / 2),
      6, cv::Scalar(0, 255, 255), cv::FILLED);

    // Tilt gauge (right side)
    int gauge_x = w - 50;
    int gauge_y = 80;
    int gauge_height = h - 160;
    int gauge_width = 25;

    cv::putText(frame, "TILT", cv::Point(gauge_x - 5, gauge_y - 10),
      cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(200, 200, 200), 1);

    // Gauge background
    cv::rectangle(frame, cv::Point(gauge_x, gauge_y),
      cv::Point(gauge_x + gauge_width, gauge_y + gauge_height),
      cv::Scalar(50, 50, 50), cv::FILLED);

    // Center line
    cv::line(frame, cv::Point(gauge_x, gauge_y + gauge_height / 2),
      cv::Point(gauge_x + gauge_width, gauge_y + gauge_height / 2),
      cv::Scalar(255, 255, 255), 1);

    // Tilt error indicator
    int tilt_pos = gauge_y + gauge_height / 2 - static_cast<int>(tilt_error * gauge_height / 2);
    tilt_pos = std::clamp(tilt_pos, gauge_y, gauge_y + gauge_height);
    cv::circle(frame, cv::Point(gauge_x + gauge_width / 2, tilt_pos),
      6, cv::Scalar(0, 255, 255), cv::FILLED);
  }

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pan_tilt_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // OpenCV / YOLO
  cv::dnn::Net net_;
  std::vector<std::string> output_layers_;
  std::vector<std::string> class_names_;
  cv::VideoCapture cap_;

  // Parameters
  double pan_kp_;
  double pan_ki_;
  double tilt_kp_;
  double tilt_ki_;
  double pan_limit_;
  double tilt_limit_;
  bool use_camera_device_;
  int camera_device_id_;
  double confidence_threshold_;
  double nms_threshold_;

  // Current joint positions
  double current_pan_ = 0.0;
  double current_tilt_ = 0.0;

  // PI control state
  double pan_integral_error_ = 0.0;
  double tilt_integral_error_ = 0.0;
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
