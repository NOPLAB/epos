#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/tracking.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <cmath>

class BodyTrackerNode : public rclcpp::Node
{
  // Tracking state enum (defined at class scope for early access)
  enum class TrackingState { DETECTING, TRACKING };

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
    declare_parameter("pan_base_threshold", 0.7);   // pan角度がこの割合を超えたら台車も回転
    declare_parameter("base_angular_gain", 0.5);    // 台車の角速度ゲイン [rad/s per rad]

    // Tracking parameters
    declare_parameter("tracking_button_index", 0);       // Aボタン
    declare_parameter("lost_threshold", 150);            // 150フレーム(約5秒)でロスト判定
    declare_parameter("yolo_verification_interval", 30); // 30フレームごとにYOLO検証
    declare_parameter("iou_threshold", 0.3);             // IoU閾値
    declare_parameter("show_window", true);              // GUIウィンドウ表示

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
    pan_base_threshold_ = get_parameter("pan_base_threshold").as_double();
    base_angular_gain_ = get_parameter("base_angular_gain").as_double();

    // Tracking parameters
    tracking_button_index_ = get_parameter("tracking_button_index").as_int();
    lost_threshold_ = get_parameter("lost_threshold").as_int();
    yolo_verification_interval_ = get_parameter("yolo_verification_interval").as_int();
    iou_threshold_ = get_parameter("iou_threshold").as_double();
    show_window_ = get_parameter("show_window").as_bool();

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

    // Publisher for base velocity commands
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive_controller/cmd_vel_unstamped", 10);

    // Subscriber for current joint state
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&BodyTrackerNode::joint_state_callback, this, std::placeholders::_1));

    // Subscriber for joystick input
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&BodyTrackerNode::joy_callback, this, std::placeholders::_1));

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
    RCLCPP_INFO(get_logger(), "Tracking: button=%d, lost_threshold=%d, yolo_interval=%d",
                tracking_button_index_, lost_threshold_, yolo_verification_interval_);
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

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (static_cast<size_t>(tracking_button_index_) >= msg->buttons.size()) {
      return;
    }

    bool current_button = msg->buttons[tracking_button_index_] != 0;

    // Edge detection (trigger on button press)
    if (current_button && !last_tracking_button_state_) {
      toggle_tracking();
    }

    last_tracking_button_state_ = current_button;
  }

  void toggle_tracking()
  {
    if (tracking_state_ == TrackingState::DETECTING) {
      // Request to start tracking (will be processed in next frame)
      request_start_tracking_ = true;
      RCLCPP_INFO(get_logger(), "Tracking start requested");
    } else {
      // Stop tracking
      stop_tracking();
      RCLCPP_INFO(get_logger(), "Tracking stopped by user");
    }
  }

  bool start_tracking(const cv::Mat& frame, const cv::Rect& bbox)
  {
    try {
      tracker_ = cv::TrackerCSRT::create();
      tracker_->init(frame, bbox);

      tracked_bbox_ = bbox;
      tracker_initialized_ = true;
      tracking_state_ = TrackingState::TRACKING;
      tracking_lost_count_ = 0;
      yolo_verification_counter_ = 0;

      RCLCPP_INFO(get_logger(), "Started tracking: bbox=(%d,%d,%d,%d)",
                  bbox.x, bbox.y, bbox.width, bbox.height);
      return true;
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to init tracker: %s", e.what());
      return false;
    }
  }

  void stop_tracking()
  {
    tracker_.release();
    tracker_initialized_ = false;
    tracking_state_ = TrackingState::DETECTING;
    tracking_lost_count_ = 0;
    yolo_verification_counter_ = 0;
    request_start_tracking_ = false;

    // Reset PI integral errors when stopping tracking
    pan_integral_error_ = 0.0;
    tilt_integral_error_ = 0.0;
  }

  bool update_tracker(const cv::Mat& frame)
  {
    if (!tracker_initialized_ || tracker_.empty()) {
      return false;
    }

    return tracker_->update(frame, tracked_bbox_);
  }

  cv::Rect select_best_target(const std::vector<cv::Rect>& boxes,
                              const std::vector<int>& indices,
                              int frame_width, int frame_height)
  {
    // Select person closest to frame center
    int center_x = frame_width / 2;
    int center_y = frame_height / 2;

    cv::Rect best_box;
    double min_dist = std::numeric_limits<double>::max();

    for (int idx : indices) {
      const cv::Rect& box = boxes[idx];
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

  double calculate_iou(const cv::Rect& a, const cv::Rect& b)
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

  struct YoloResult {
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> indices;
  };

  YoloResult run_yolo_detection(const cv::Mat& frame)
  {
    YoloResult result;
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

          result.boxes.push_back(cv::Rect(x, y, w, h));
          result.confidences.push_back(static_cast<float>(confidence));
        }
      }
    }

    // Non-maximum suppression
    cv::dnn::NMSBoxes(result.boxes, result.confidences,
                      confidence_threshold_, nms_threshold_, result.indices);

    return result;
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
    int frame_center_x = width / 2;
    int frame_center_y = height / 2;

    cv::Rect target_box;
    bool target_valid = false;
    float target_confidence = 0.0;
    YoloResult yolo_result;
    size_t num_detections = 0;

    if (tracking_state_ == TrackingState::TRACKING) {
      // === TRACKING MODE ===
      bool track_success = update_tracker(frame);
      yolo_verification_counter_++;
      if (yolo_verification_counter_ >= yolo_verification_interval_) {
        yolo_verification_counter_ = 0;

        yolo_result = run_yolo_detection(frame);
        num_detections = yolo_result.indices.size();

        // Find best matching YOLO detection
        double best_iou = 0.0;
        cv::Rect best_match;
        for (int idx : yolo_result.indices) {
          double iou = calculate_iou(tracked_bbox_, yolo_result.boxes[idx]);
          if (iou > best_iou) {
            best_iou = iou;
            best_match = yolo_result.boxes[idx];
          }
        }

        // If YOLO detection matches, update tracker with corrected bbox
        if (best_iou > iou_threshold_) {
          // Re-initialize tracker with YOLO-corrected bbox
          try {
            tracker_->init(frame, best_match);
            tracked_bbox_ = best_match;
          } catch (const cv::Exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to reinit tracker: %s", e.what());
          }
        }
      }

      if (track_success) {
        tracking_lost_count_ = 0;
        target_box = tracked_bbox_;
        target_valid = true;
      } else {
        tracking_lost_count_++;
        if (tracking_lost_count_ >= lost_threshold_) {
          RCLCPP_WARN(get_logger(), "Target lost, returning to detection mode");
          stop_tracking();
        } else {
          // Use last known position
          target_box = tracked_bbox_;
          target_valid = true;
        }
      }

    } else {
      // === DETECTING MODE ===
      yolo_result = run_yolo_detection(frame);
      num_detections = yolo_result.indices.size();

      if (!yolo_result.indices.empty()) {
        if (request_start_tracking_) {
          // Select target closest to center and start tracking
          cv::Rect selected = select_best_target(yolo_result.boxes, yolo_result.indices, width, height);
          if (start_tracking(frame, selected)) {
            target_box = selected;
            target_valid = true;
          }
          request_start_tracking_ = false;
        } else {
          // Normal detecting mode: follow most confident person
          float best_confidence = 0.0;
          for (int idx : yolo_result.indices) {
            if (yolo_result.confidences[idx] > best_confidence) {
              best_confidence = yolo_result.confidences[idx];
              target_box = yolo_result.boxes[idx];
              target_confidence = best_confidence;
              target_valid = true;
            }
          }
        }
      } else {
        // No detection and start tracking was requested
        if (request_start_tracking_) {
          RCLCPP_WARN(get_logger(), "No person detected, cannot start tracking");
          request_start_tracking_ = false;
        }
      }
    }

    // === PI Control ===
    double pan_error = 0.0;
    double tilt_error = 0.0;
    double pan_p_term = 0.0, pan_i_term = 0.0;
    double tilt_p_term = 0.0, tilt_i_term = 0.0;
    double target_pan = current_pan_;
    double target_tilt = current_tilt_;

    if (target_valid) {
      int body_center_x = target_box.x + target_box.width / 2;
      int body_center_y = target_box.y + target_box.height / 2;

      pan_error = static_cast<double>(frame_center_x - body_center_x) / frame_center_x;
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
      double tilt_delta = -(tilt_p_term + tilt_i_term);

      target_pan = current_pan_ + pan_delta;
      target_tilt = current_tilt_ + tilt_delta;
      target_pan = std::clamp(target_pan, -pan_limit_, pan_limit_);
      target_tilt = std::clamp(target_tilt, -tilt_limit_, tilt_limit_);

      // Draw target
      int body_center_x_draw = target_box.x + target_box.width / 2;
      int body_center_y_draw = target_box.y + target_box.height / 2;

      if (tracking_state_ == TrackingState::TRACKING) {
        // Orange thick box for tracking mode
        cv::rectangle(frame, target_box, cv::Scalar(0, 165, 255), 3);
      } else {
        // Green box for detecting mode
        cv::rectangle(frame, target_box, cv::Scalar(0, 255, 0), 2);
        if (target_confidence > 0) {
          char conf_buf[32];
          snprintf(conf_buf, sizeof(conf_buf), "%.1f%%", target_confidence * 100);
          cv::putText(frame, conf_buf, cv::Point(target_box.x, target_box.y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
      }

      cv::drawMarker(frame, cv::Point(body_center_x_draw, body_center_y_draw),
        cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
      cv::line(frame, cv::Point(frame_center_x, frame_center_y),
        cv::Point(body_center_x_draw, body_center_y_draw), cv::Scalar(255, 255, 0), 2);

    } else {
      pan_integral_error_ *= 0.95;
      tilt_integral_error_ *= 0.95;
    }

    // Draw all detected persons in detecting mode (faded)
    if (tracking_state_ == TrackingState::DETECTING) {
      for (int idx : yolo_result.indices) {
        if (yolo_result.boxes[idx] != target_box) {
          cv::rectangle(frame, yolo_result.boxes[idx], cv::Scalar(100, 100, 100), 1);
        }
      }
    }

    // Publish pan-tilt command
    auto pan_tilt_msg = std_msgs::msg::Float64MultiArray();
    pan_tilt_msg.data.resize(2);
    pan_tilt_msg.data[0] = target_pan;
    pan_tilt_msg.data[1] = target_tilt;
    pan_tilt_pub_->publish(pan_tilt_msg);

    // Calculate base angular velocity
    double base_angular_vel = 0.0;
    double pan_ratio = std::abs(current_pan_) / pan_limit_;
    if (pan_ratio > pan_base_threshold_ && target_valid) {
      double excess_ratio = (pan_ratio - pan_base_threshold_) / (1.0 - pan_base_threshold_);
      base_angular_vel = std::copysign(excess_ratio * base_angular_gain_, current_pan_);
    }

    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    cmd_vel_msg.angular.z = base_angular_vel;
    cmd_vel_pub_->publish(cmd_vel_msg);

    // === Draw UI ===
    if (show_window_) {
      draw_ui(frame, target_valid, pan_error, tilt_error,
              pan_p_term, pan_i_term, tilt_p_term, tilt_i_term,
              target_pan, target_tilt, base_angular_vel, num_detections);

      cv::imshow("Body Tracker", frame);
      cv::waitKey(1);
    }
  }

  void draw_ui(cv::Mat& frame, bool body_detected,
               double pan_error, double tilt_error,
               double pan_p, double pan_i, double tilt_p, double tilt_i,
               double target_pan, double target_tilt,
               double base_angular_vel, size_t num_detections)
  {
    int w = frame.cols;
    int h = frame.rows;

    // Draw crosshair at center
    cv::line(frame, cv::Point(w / 2, 0), cv::Point(w / 2, h),
      cv::Scalar(100, 100, 100), 1);
    cv::line(frame, cv::Point(0, h / 2), cv::Point(w, h / 2),
      cv::Scalar(100, 100, 100), 1);

    // Status panel background
    cv::rectangle(frame, cv::Point(10, 10), cv::Point(280, 260),
      cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(frame, cv::Point(10, 10), cv::Point(280, 260),
      cv::Scalar(100, 100, 100), 1);

    // Status text - show tracking state
    cv::Scalar status_color;
    std::string status_text;
    std::string mode_text;

    if (tracking_state_ == TrackingState::TRACKING) {
      status_color = cv::Scalar(0, 165, 255);  // Orange
      status_text = "TRACKING";
      mode_text = "NanoTrack";
    } else if (body_detected) {
      status_color = cv::Scalar(0, 255, 0);    // Green
      status_text = "DETECTING";
      mode_text = "YOLO v4-tiny";
    } else {
      status_color = cv::Scalar(0, 0, 255);    // Red
      status_text = "SEARCHING";
      mode_text = "YOLO v4-tiny";
    }

    cv::putText(frame, status_text, cv::Point(20, 35),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2);

    // Detection/Tracking method
    cv::putText(frame, mode_text, cv::Point(140, 35),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

    // Detection count and tracking info
    char buf[64];
    if (tracking_state_ == TrackingState::TRACKING) {
      snprintf(buf, sizeof(buf), "Lost: %d/%d", tracking_lost_count_, lost_threshold_);
    } else {
      snprintf(buf, sizeof(buf), "Persons: %zu", num_detections);
    }
    cv::putText(frame, buf, cv::Point(20, 55),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

    // Button hint (top right)
    std::string hint = (tracking_state_ == TrackingState::DETECTING) ?
      "[A] Start Track" : "[A] Stop Track";
    cv::putText(frame, hint, cv::Point(w - 130, 25),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);

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

    // Base control info
    cv::putText(frame, "--- BASE ---", cv::Point(20, 200),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

    double pan_ratio = std::abs(current_pan_) / pan_limit_;
    snprintf(buf, sizeof(buf), "Pan ratio: %.1f%% (thresh: %.0f%%)",
             pan_ratio * 100, pan_base_threshold_ * 100);
    cv::putText(frame, buf, cv::Point(20, 218),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);

    snprintf(buf, sizeof(buf), "Angular vel: %+.3f rad/s", base_angular_vel);
    cv::Scalar vel_color = std::abs(base_angular_vel) > 0.01 ?
      cv::Scalar(0, 255, 255) : cv::Scalar(200, 200, 200);
    cv::putText(frame, buf, cv::Point(20, 236),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, vel_color, 1);

    // Gains
    snprintf(buf, sizeof(buf), "Pan Kp=%.4f Ki=%.5f", pan_kp_, pan_ki_);
    cv::putText(frame, buf, cv::Point(20, 253),
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // OpenCV / YOLO
  cv::dnn::Net net_;
  std::vector<std::string> output_layers_;
  std::vector<std::string> class_names_;
  cv::VideoCapture cap_;

  // Tracker
  cv::Ptr<cv::Tracker> tracker_;
  cv::Rect tracked_bbox_;
  bool tracker_initialized_ = false;
  TrackingState tracking_state_ = TrackingState::DETECTING;
  bool request_start_tracking_ = false;

  // Joystick state
  bool last_tracking_button_state_ = false;

  // Tracking lost detection
  int tracking_lost_count_ = 0;
  int yolo_verification_counter_ = 0;

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
  double pan_base_threshold_;
  double base_angular_gain_;

  // Tracking parameters
  int tracking_button_index_;
  int lost_threshold_;
  int yolo_verification_interval_;
  double iou_threshold_;
  bool show_window_;

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
