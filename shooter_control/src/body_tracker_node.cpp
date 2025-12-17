#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
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
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>

class BodyTrackerNode : public rclcpp::Node
{
  // Tracking state enum (defined at class scope for early access)
  enum class TrackingState { DETECTING, TRACKING };

  // Scan direction enum
  enum class ScanDirection { RIGHT, LEFT };

  // Firing state enum for auto-fire sequence
  enum class FiringState { IDLE, SPINUP, FIRING, COOLDOWN };

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
    declare_parameter("velocity_feedforward_gain", 0.005); // 目標速度フィードフォワードゲイン

    // Tracking parameters
    declare_parameter("tracking_button_index", 0);       // Aボタン
    declare_parameter("lost_threshold", 150);            // 150フレーム(約5秒)でロスト判定
    declare_parameter("verification_interval", 30);      // 30フレームごとに検証（約1秒@30fps）
    declare_parameter("iou_threshold", 0.3);             // IoU閾値
    declare_parameter("detection_miss_threshold", 3);    // 連続N回検出失敗でDETECTINGに移行
    declare_parameter("show_window", true);              // GUIウィンドウ表示

    // Model parameters
    declare_parameter("detection_skip_frames", 3);       // DETECTINGモードでのフレームスキップ数

    // Scan parameters (DETECTING mode) - set based on homing center offset
    declare_parameter("scan_enabled", true);             // スキャン動作を有効化
    declare_parameter("scan_pan_velocity", 0.5);         // パンスキャン速度 (rad/s)
    declare_parameter("scan_tilt_velocity", 0.3);        // チルトスキャン速度 (rad/s)
    declare_parameter("scan_pan_min", -1.5);             // パンスキャン最小角度 (rad)
    declare_parameter("scan_pan_max", 1.5);              // パンスキャン最大角度 (rad)
    declare_parameter("scan_tilt_min", -0.5);            // チルト最小角度 (rad)
    declare_parameter("scan_tilt_max", 0.3);             // チルト最大角度 (rad)
    declare_parameter("scan_tilt_step", 0.3);            // チルトステップ (rad)

    // Auto-fire parameters
    declare_parameter("auto_fire_enabled", false);       // 自動射撃を有効化
    declare_parameter("auto_fire_threshold", 0.7);       // 射撃しきい値（認識信頼度）
    declare_parameter("auto_fire_cooldown_ms", 2000);    // 射撃クールダウン (ms)
    declare_parameter("auto_fire_center_tolerance", 0.15); // 中央許容範囲（正規化）
    declare_parameter("shooter_speed_rpm", 10000.0);     // シューター速度 (RPM)
    declare_parameter("shooter_spinup_ms", 500);         // スピンアップ時間 (ms)
    declare_parameter("shooter_fire_duration_ms", 300);  // 発射時間 (ms)

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
    velocity_feedforward_gain_ = get_parameter("velocity_feedforward_gain").as_double();

    // Tracking parameters
    tracking_button_index_ = get_parameter("tracking_button_index").as_int();
    lost_threshold_ = get_parameter("lost_threshold").as_int();
    verification_interval_ = get_parameter("verification_interval").as_int();
    iou_threshold_ = get_parameter("iou_threshold").as_double();
    detection_miss_threshold_ = get_parameter("detection_miss_threshold").as_int();
    show_window_ = get_parameter("show_window").as_bool();

    // Model parameters
    detection_skip_frames_ = get_parameter("detection_skip_frames").as_int();

    // Scan parameters
    scan_enabled_ = get_parameter("scan_enabled").as_bool();
    scan_pan_velocity_ = get_parameter("scan_pan_velocity").as_double();
    scan_tilt_velocity_ = get_parameter("scan_tilt_velocity").as_double();
    scan_pan_min_ = get_parameter("scan_pan_min").as_double();
    scan_pan_max_ = get_parameter("scan_pan_max").as_double();
    scan_tilt_min_ = get_parameter("scan_tilt_min").as_double();
    scan_tilt_max_ = get_parameter("scan_tilt_max").as_double();
    scan_tilt_step_ = get_parameter("scan_tilt_step").as_double();

    // Auto-fire parameters
    auto_fire_enabled_ = get_parameter("auto_fire_enabled").as_bool();
    auto_fire_threshold_ = get_parameter("auto_fire_threshold").as_double();
    auto_fire_cooldown_ms_ = get_parameter("auto_fire_cooldown_ms").as_int();
    auto_fire_center_tolerance_ = get_parameter("auto_fire_center_tolerance").as_double();
    shooter_speed_rpm_ = get_parameter("shooter_speed_rpm").as_double();
    shooter_spinup_ms_ = get_parameter("shooter_spinup_ms").as_int();
    shooter_fire_duration_ms_ = get_parameter("shooter_fire_duration_ms").as_int();
    // Convert RPM to rad/s for ros2_control
    shooter_speed_rad_s_ = shooter_speed_rpm_ * 2.0 * M_PI / 60.0;

    // Load MobileNet-SSD model
    std::string pkg_share = ament_index_cpp::get_package_share_directory("shooter_control");
    std::string prototxt_path = pkg_share + "/models/MobileNetSSD_deploy.prototxt";
    std::string caffemodel_path = pkg_share + "/models/MobileNetSSD_deploy.caffemodel";

    RCLCPP_INFO(get_logger(), "Loading MobileNet-SSD model from: %s", pkg_share.c_str());

    net_ = cv::dnn::readNetFromCaffe(prototxt_path, caffemodel_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    RCLCPP_INFO(get_logger(), "MobileNet-SSD model loaded successfully");

    // Publisher for pan-tilt velocity commands (rad/s)
    pan_tilt_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pan_tilt_controller/commands", 10);

    // Publisher for base velocity commands
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive_controller/cmd_vel_unstamped", 10);

    // Publisher for shooter motor commands
    shooter_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/shooter_controller/commands", 10);

    // Subscriber for current joint state
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&BodyTrackerNode::joint_state_callback, this, std::placeholders::_1));

    // Subscriber for joystick input
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&BodyTrackerNode::joy_callback, this, std::placeholders::_1));

    // Service client for trigger (auto-fire)
    if (auto_fire_enabled_) {
      trigger_client_ = create_client<std_srvs::srv::Trigger>("/servo_trigger_node/trigger");
      RCLCPP_INFO(get_logger(), "Auto-fire enabled: threshold=%.2f, cooldown=%dms, center_tol=%.2f",
                  auto_fire_threshold_, auto_fire_cooldown_ms_, auto_fire_center_tolerance_);
    }

    if (use_camera_device_) {
      // Use OpenCV VideoCapture directly
      cap_.open(camera_device_id_);
      if (!cap_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera device %d", camera_device_id_);
        throw std::runtime_error("Failed to open camera");
      }
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

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
    RCLCPP_INFO(get_logger(), "Tracking: button=%d, lost_threshold=%d, verification_interval=%d, miss_threshold=%d",
                tracking_button_index_, lost_threshold_, verification_interval_, detection_miss_threshold_);

    // Start YOLO thread
    yolo_thread_running_ = true;
    yolo_thread_ = std::thread(&BodyTrackerNode::yolo_thread_func, this);
    RCLCPP_INFO(get_logger(), "Async YOLO detection thread started");
  }

  ~BodyTrackerNode()
  {
    // Signal YOLO thread to stop
    yolo_thread_running_ = false;
    yolo_cv_.notify_all();
    if (yolo_thread_.joinable()) {
      yolo_thread_.join();
    }

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
    tracking_enabled_ = !tracking_enabled_;

    if (tracking_enabled_) {
      // 自動スキャン＆検出開始（DETECTINGモードへ）
      tracking_state_ = TrackingState::DETECTING;
      reset_scan_state();
      RCLCPP_INFO(get_logger(), "Tracking ENABLED - scanning for target");
    } else {
      // 完全停止
      stop_tracking();
      RCLCPP_INFO(get_logger(), "Tracking DISABLED");
    }
  }

  cv::Ptr<cv::TrackerKCF> create_kcf_tracker()
  {
    cv::TrackerKCF::Params params;
    params.detect_thresh = 0.4;  // 検出閾値を下げて高速化
    params.resize = true;        // 内部リサイズを有効化
    return cv::TrackerKCF::create(params);
  }

  // KCF用にフレームをリサイズ（高速化のため）
  cv::Mat resize_for_kcf(const cv::Mat& frame)
  {
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(), kcf_scale_, kcf_scale_, cv::INTER_LINEAR);
    return resized;
  }

  // bboxをKCFスケールに変換
  cv::Rect scale_bbox_for_kcf(const cv::Rect& bbox)
  {
    return cv::Rect(
      static_cast<int>(bbox.x * kcf_scale_),
      static_cast<int>(bbox.y * kcf_scale_),
      static_cast<int>(bbox.width * kcf_scale_),
      static_cast<int>(bbox.height * kcf_scale_)
    );
  }

  // bboxを元のスケールに戻す
  cv::Rect scale_bbox_from_kcf(const cv::Rect& bbox)
  {
    return cv::Rect(
      static_cast<int>(bbox.x / kcf_scale_),
      static_cast<int>(bbox.y / kcf_scale_),
      static_cast<int>(bbox.width / kcf_scale_),
      static_cast<int>(bbox.height / kcf_scale_)
    );
  }

  bool start_tracking(const cv::Mat& frame, const cv::Rect& bbox)
  {
    try {
      tracker_ = create_kcf_tracker();

      // KCF用にリサイズしたフレームとbboxで初期化
      cv::Mat kcf_frame = resize_for_kcf(frame);
      cv::Rect kcf_bbox = scale_bbox_for_kcf(bbox);
      tracker_->init(kcf_frame, kcf_bbox);

      tracked_bbox_ = bbox;  // 元のスケールで保存
      tracker_initialized_ = true;
      tracking_state_ = TrackingState::TRACKING;
      tracking_lost_count_ = 0;
      verification_counter_ = 0;
      detection_miss_count_ = 0;

      RCLCPP_INFO(get_logger(), "Started tracking: bbox=(%d,%d,%d,%d) kcf_scale=%.2f",
                  bbox.x, bbox.y, bbox.width, bbox.height, kcf_scale_);
      return true;
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to init tracker: %s", e.what());
      return false;
    }
  }

  // トラッキングロスト時: DETECTINGモードに戻る（スキャン継続）
  void return_to_detecting()
  {
    tracker_.release();
    tracker_initialized_ = false;
    tracking_state_ = TrackingState::DETECTING;
    tracking_lost_count_ = 0;
    verification_counter_ = 0;
    detection_miss_count_ = 0;
    detection_in_progress_ = false;

    // Reset PI integral errors
    pan_integral_error_ = 0.0;
    tilt_integral_error_ = 0.0;

    // Reset velocity feedforward state
    prev_body_center_x_ = -1;
    prev_body_center_y_ = -1;

    // スキャン状態をリセットして再スキャン開始
    reset_scan_state();

    RCLCPP_INFO(get_logger(), "Returning to DETECTING mode, resuming scan");
  }

  // 完全停止: ユーザーがAボタンで無効化した時
  void stop_tracking()
  {
    return_to_detecting();
    tracking_enabled_ = false;

    // 射撃中なら停止
    if (firing_state_ != FiringState::IDLE) {
      set_shooter_velocity(0.0);
      firing_state_ = FiringState::IDLE;
    }
  }

  void reset_scan_state()
  {
    scan_direction_ = ScanDirection::RIGHT;
    scan_target_tilt_ = scan_tilt_min_;
  }

  // スキャン速度を計算（DETECTINGモード用）
  void calculate_scan_velocity(double& pan_vel, double& tilt_vel)
  {
    if (!scan_enabled_) {
      pan_vel = 0.0;
      tilt_vel = 0.0;
      return;
    }

    // パンの端に到達したら方向を反転し、チルトを1段階変更
    if (scan_direction_ == ScanDirection::RIGHT) {
      if (current_pan_ >= scan_pan_max_) {
        // 右端(max)に到達 → 左に方向転換、チルトを上げる
        scan_direction_ = ScanDirection::LEFT;
        scan_target_tilt_ += scan_tilt_step_;
        if (scan_target_tilt_ > scan_tilt_max_) {
          scan_target_tilt_ = scan_tilt_min_;  // 最上部に達したら最下部に戻る
        }
        RCLCPP_DEBUG(get_logger(), "Scan: reached max (%.2f), switching to LEFT, tilt=%.2f",
                     scan_pan_max_, scan_target_tilt_);
      }
      pan_vel = scan_pan_velocity_;
    } else {
      if (current_pan_ <= scan_pan_min_) {
        // 左端(min)に到達 → 右に方向転換、チルトを上げる
        scan_direction_ = ScanDirection::RIGHT;
        scan_target_tilt_ += scan_tilt_step_;
        if (scan_target_tilt_ > scan_tilt_max_) {
          scan_target_tilt_ = scan_tilt_min_;
        }
        RCLCPP_DEBUG(get_logger(), "Scan: reached min (%.2f), switching to RIGHT, tilt=%.2f",
                     scan_pan_min_, scan_target_tilt_);
      }
      pan_vel = -scan_pan_velocity_;
    }

    // チルトは目標に向かってP制御
    double tilt_error = scan_target_tilt_ - current_tilt_;
    tilt_vel = std::clamp(tilt_error * 2.0, -scan_tilt_velocity_, scan_tilt_velocity_);
  }

  // シューターモーターを制御
  void set_shooter_velocity(double velocity_rad_s)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {velocity_rad_s};
    shooter_pub_->publish(msg);
  }

  // 射撃シーケンス状態マシンを更新
  void update_firing_sequence()
  {
    if (!auto_fire_enabled_) {
      return;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - firing_state_start_time_);

    switch (firing_state_) {
      case FiringState::IDLE:
        // IDLEでは何もしない（try_auto_fireで開始）
        break;

      case FiringState::SPINUP:
        // スピンアップ完了を待つ
        if (elapsed.count() >= shooter_spinup_ms_) {
          // サーボトリガーを発動
          if (trigger_client_) {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            trigger_client_->async_send_request(request);
            RCLCPP_INFO(get_logger(), "Servo triggered");
          }
          firing_state_ = FiringState::FIRING;
          firing_state_start_time_ = now;
        }
        break;

      case FiringState::FIRING:
        // 発射時間経過を待つ
        if (elapsed.count() >= shooter_fire_duration_ms_) {
          // シューター停止
          set_shooter_velocity(0.0);
          RCLCPP_INFO(get_logger(), "Shooter stopped, entering cooldown");
          firing_state_ = FiringState::COOLDOWN;
          firing_state_start_time_ = now;
          last_fire_time_ = now;
        }
        break;

      case FiringState::COOLDOWN:
        // クールダウン完了を待つ
        if (elapsed.count() >= auto_fire_cooldown_ms_) {
          firing_state_ = FiringState::IDLE;
          RCLCPP_DEBUG(get_logger(), "Firing cooldown complete, ready for next shot");
        }
        break;
    }
  }

  // 自動射撃を試行（条件: 認識信頼度 > しきい値、ターゲットが中央付近、IDLE状態）
  void try_auto_fire(float confidence, double pan_error, double tilt_error)
  {
    if (!auto_fire_enabled_) {
      return;
    }

    // 射撃中は新しい射撃を開始しない
    if (firing_state_ != FiringState::IDLE) {
      return;
    }

    // 認識信頼度チェック
    if (confidence < auto_fire_threshold_) {
      return;
    }

    // ターゲットが中央付近かチェック（pan/tilt誤差が許容範囲内）
    if (std::abs(pan_error) > auto_fire_center_tolerance_ ||
        std::abs(tilt_error) > auto_fire_center_tolerance_) {
      return;
    }

    // 射撃シーケンス開始
    RCLCPP_INFO(get_logger(), "AUTO-FIRE START! conf=%.2f, pan_err=%.3f, tilt_err=%.3f",
                confidence, pan_error, tilt_error);

    // シューターモーター開始
    set_shooter_velocity(shooter_speed_rad_s_);
    RCLCPP_INFO(get_logger(), "Shooter motor started: %.0f RPM (%.2f rad/s)",
                shooter_speed_rpm_, shooter_speed_rad_s_);

    firing_state_ = FiringState::SPINUP;
    firing_state_start_time_ = std::chrono::steady_clock::now();
  }

  bool update_tracker(const cv::Mat& frame)
  {
    if (!tracker_initialized_ || tracker_.empty()) {
      return false;
    }

    // KCF用にリサイズしたフレームで更新
    cv::Mat kcf_frame = resize_for_kcf(frame);
    cv::Rect kcf_bbox = scale_bbox_for_kcf(tracked_bbox_);

    bool success = tracker_->update(kcf_frame, kcf_bbox);

    if (success) {
      // 結果を元のスケールに戻す
      tracked_bbox_ = scale_bbox_from_kcf(kcf_bbox);
    }

    return success;
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

  struct DetectionResult {
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> indices;
  };

  // MobileNet-SSD detection
  DetectionResult run_detection(const cv::Mat& frame)
  {
    DetectionResult result;
    int width = frame.cols;
    int height = frame.rows;

    // Create blob from image (MobileNet-SSD uses 300x300 input)
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 0.007843, cv::Size(300, 300),
                           cv::Scalar(127.5, 127.5, 127.5), false, false);
    net_.setInput(blob);

    // Forward pass
    cv::Mat detection = net_.forward();

    // Parse detections
    // Output shape: [1, 1, N, 7] where each detection is [batch_id, class_id, confidence, x1, y1, x2, y2]
    cv::Mat detections(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    for (int i = 0; i < detections.rows; i++) {
      float confidence = detections.at<float>(i, 2);
      int class_id = static_cast<int>(detections.at<float>(i, 1));

      // Only detect "person" with sufficient confidence
      if (class_id == ssd_person_class_id_ && confidence > confidence_threshold_) {
        int x1 = static_cast<int>(detections.at<float>(i, 3) * width);
        int y1 = static_cast<int>(detections.at<float>(i, 4) * height);
        int x2 = static_cast<int>(detections.at<float>(i, 5) * width);
        int y2 = static_cast<int>(detections.at<float>(i, 6) * height);

        // Clamp to frame bounds
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

    // Non-maximum suppression
    cv::dnn::NMSBoxes(result.boxes, result.confidences,
                      confidence_threshold_, nms_threshold_, result.indices);

    return result;
  }

  // Async detection thread function
  void yolo_thread_func()
  {
    while (yolo_thread_running_) {
      cv::Mat frame_to_process;

      // Wait for a frame to process
      {
        std::unique_lock<std::mutex> lock(yolo_mutex_);
        yolo_cv_.wait(lock, [this] {
          return !yolo_thread_running_ || yolo_request_pending_;
        });

        if (!yolo_thread_running_) {
          break;
        }

        frame_to_process = yolo_frame_.clone();
        yolo_request_pending_ = false;
      }

      // Run detection (blocking but in separate thread)
      DetectionResult result = run_detection(frame_to_process);

      // Store results
      {
        std::lock_guard<std::mutex> lock(yolo_result_mutex_);
        yolo_async_result_ = result;
        yolo_result_ready_ = true;
      }
    }
  }

  // Request async YOLO detection
  void request_async_yolo(const cv::Mat& frame)
  {
    std::lock_guard<std::mutex> lock(yolo_mutex_);
    yolo_frame_ = frame.clone();
    yolo_request_pending_ = true;
    yolo_cv_.notify_one();
  }

  // Check if async YOLO result is ready and retrieve it
  bool get_async_yolo_result(DetectionResult& result)
  {
    std::lock_guard<std::mutex> lock(yolo_result_mutex_);
    if (yolo_result_ready_) {
      result = yolo_async_result_;
      yolo_result_ready_ = false;
      return true;
    }
    return false;
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
    // 射撃シーケンス状態マシンを更新（毎フレーム呼び出し）
    update_firing_sequence();

    int width = frame.cols;
    int height = frame.rows;
    int frame_center_x = width / 2;
    int frame_center_y = height / 2;

    cv::Rect target_box;
    bool target_valid = false;
    float target_confidence = 0.0;
    DetectionResult yolo_result;
    size_t num_detections = 0;

    if (tracking_state_ == TrackingState::TRACKING) {
      // === TRACKING MODE ===
      // KCF is executed every frame (fast)
      bool track_success = update_tracker(frame);

      // Request async detection verification periodically (non-blocking)
      verification_counter_++;
      if (verification_counter_ >= verification_interval_) {
        verification_counter_ = 0;
        request_async_yolo(frame);
      }

      // Check if async detection result is ready (non-blocking)
      DetectionResult async_result;
      if (get_async_yolo_result(async_result)) {
        num_detections = async_result.indices.size();

        if (async_result.indices.empty()) {
          // No person detected by MobileNet-SSD
          detection_miss_count_++;
          RCLCPP_DEBUG(get_logger(), "Detection miss: %d/%d", detection_miss_count_, detection_miss_threshold_);

          if (detection_miss_count_ >= detection_miss_threshold_) {
            RCLCPP_WARN(get_logger(), "Detection verification failed %d times, returning to detection mode",
                        detection_miss_count_);
            return_to_detecting();
          }
        } else {
          // Find best matching detection
          double best_iou = 0.0;
          cv::Rect best_match;
          float best_confidence = 0.0f;
          for (int idx : async_result.indices) {
            double iou = calculate_iou(tracked_bbox_, async_result.boxes[idx]);
            if (iou > best_iou) {
              best_iou = iou;
              best_match = async_result.boxes[idx];
              best_confidence = async_result.confidences[idx];
            }
          }

          // If detection matches, reset miss count and correct tracker
          if (best_iou > iou_threshold_) {
            detection_miss_count_ = 0;  // Reset miss count on successful match
            last_detection_confidence_ = best_confidence;  // 射撃判定用に保存
            try {
              tracker_ = create_kcf_tracker();
              // KCF用にリサイズしたフレームとbboxで初期化
              cv::Mat kcf_frame = resize_for_kcf(frame);
              cv::Rect kcf_bbox = scale_bbox_for_kcf(best_match);
              tracker_->init(kcf_frame, kcf_bbox);
              tracked_bbox_ = best_match;  // 元のスケールで保存
              RCLCPP_DEBUG(get_logger(), "Tracker corrected (IoU=%.2f, conf=%.2f)", best_iou, best_confidence);
            } catch (const cv::Exception& e) {
              RCLCPP_WARN(get_logger(), "Failed to reinit tracker: %s", e.what());
            }
          } else {
            // Person detected but IoU too low (tracking wrong target?)
            detection_miss_count_++;
            RCLCPP_DEBUG(get_logger(), "IoU too low (%.2f), miss: %d/%d",
                        best_iou, detection_miss_count_, detection_miss_threshold_);

            if (detection_miss_count_ >= detection_miss_threshold_) {
              RCLCPP_WARN(get_logger(), "Tracking target mismatch, returning to detection mode");
              return_to_detecting();
            }
          }
        }
      }

      // Always use KCF result for control (not waiting for detection)
      if (track_success) {
        tracking_lost_count_ = 0;
        target_box = tracked_bbox_;
        target_valid = true;
      } else {
        tracking_lost_count_++;
        if (tracking_lost_count_ >= lost_threshold_) {
          RCLCPP_WARN(get_logger(), "KCF tracking lost, returning to detection mode");
          return_to_detecting();
        } else {
          // Use last known position
          target_box = tracked_bbox_;
          target_valid = true;
        }
      }

    } else if (tracking_enabled_) {
      // === DETECTING MODE (自動スキャン＆検出→トラッキング開始) ===

      // フレームスキップ: 指定フレーム数ごとに検出をリクエスト
      detection_frame_counter_++;
      if (detection_frame_counter_ >= detection_skip_frames_) {
        detection_frame_counter_ = 0;

        // 非同期検出をリクエスト（まだリクエスト中でなければ）
        if (!detection_in_progress_) {
          request_async_yolo(frame);
          detection_in_progress_ = true;
        }
      }

      // 非同期YOLO結果を確認（ノンブロッキング）
      DetectionResult async_result;
      if (get_async_yolo_result(async_result)) {
        detection_in_progress_ = false;  // 次のリクエストを許可
        yolo_result = async_result;
        num_detections = yolo_result.indices.size();

        if (!yolo_result.indices.empty()) {
          // 人物検出！自動的にトラッキング開始
          cv::Rect selected = select_best_target(yolo_result.boxes, yolo_result.indices, width, height);
          if (start_tracking(frame, selected)) {
            target_box = selected;
            target_valid = true;
            RCLCPP_INFO(get_logger(), "Person detected, auto-starting tracking");
          }
        }
        // 検出なしの場合はスキャン継続（何もしない）
      }
      // YOLO結果待ち中はtarget_valid=falseのまま（制御コマンドは0を送信）
    }
    // === IDLE MODE: tracking_enabled_==false の場合はYOLO検出をスキップ ===

    // === PI Control ===
    double pan_error = 0.0;
    double tilt_error = 0.0;
    double pan_p_term = 0.0, pan_i_term = 0.0;
    double tilt_p_term = 0.0, tilt_i_term = 0.0;
    double target_pan = current_pan_;
    double target_tilt = current_tilt_;

    // Velocity commands (rad/s) for pan_tilt_controller
    double pan_velocity_cmd = 0.0;
    double tilt_velocity_cmd = 0.0;

    if (!tracking_enabled_) {
      // === IDLE: 完全停止（YOLO検出もスキップ済み） ===
      pan_velocity_cmd = 0.0;
      tilt_velocity_cmd = 0.0;

      target_pan = current_pan_;
      target_tilt = current_tilt_;

      // 積分誤差をリセット
      pan_integral_error_ = 0.0;
      tilt_integral_error_ = 0.0;

    } else if (target_valid) {
      int body_center_x = target_box.x + target_box.width / 2;
      int body_center_y = target_box.y + target_box.height / 2;

      // Calculate target velocity (pixels/frame -> normalized)
      double target_vel_x = 0.0;
      double target_vel_y = 0.0;
      if (prev_body_center_x_ >= 0 && prev_body_center_y_ >= 0) {
        target_vel_x = static_cast<double>(body_center_x - prev_body_center_x_) / frame_center_x;
        target_vel_y = static_cast<double>(body_center_y - prev_body_center_y_) / frame_center_y;
      }
      prev_body_center_x_ = body_center_x;
      prev_body_center_y_ = body_center_y;

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

      // Velocity feedforward: target moving right -> pan should follow (negative direction)
      double pan_feedforward = -velocity_feedforward_gain_ * target_vel_x;
      double tilt_feedforward = -velocity_feedforward_gain_ * target_vel_y;

      // Set velocity commands: PI control + velocity feedforward
      pan_velocity_cmd = pan_delta + pan_feedforward;
      tilt_velocity_cmd = tilt_delta + tilt_feedforward;

      // TRACKINGモードで自動射撃を試行
      if (tracking_state_ == TrackingState::TRACKING) {
        try_auto_fire(last_detection_confidence_, pan_error, tilt_error);
      }

      // Update target position for display/monitoring only
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

      // DETECTINGモードかつ追跡有効時はスキャン動作
      if (tracking_enabled_ && tracking_state_ == TrackingState::DETECTING) {
        calculate_scan_velocity(pan_velocity_cmd, tilt_velocity_cmd);
      }
    }

    // Draw all detected persons in detecting mode (faded) - only when tracking is enabled
    if (tracking_enabled_ && tracking_state_ == TrackingState::DETECTING) {
      for (int idx : yolo_result.indices) {
        if (yolo_result.boxes[idx] != target_box) {
          cv::rectangle(frame, yolo_result.boxes[idx], cv::Scalar(100, 100, 100), 1);
        }
      }
    }

    // Publish pan-tilt velocity command (rad/s)
    auto pan_tilt_msg = std_msgs::msg::Float64MultiArray();
    pan_tilt_msg.data.resize(2);
    pan_tilt_msg.data[0] = pan_velocity_cmd;
    pan_tilt_msg.data[1] = tilt_velocity_cmd;
    pan_tilt_pub_->publish(pan_tilt_msg);

    // Log control output for debugging
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "Control: pan_vel=%.4f tilt_vel=%.4f target_valid=%d state=%s",
      pan_velocity_cmd, tilt_velocity_cmd, target_valid,
      tracking_state_ == TrackingState::TRACKING ? "TRACKING" : "DETECTING");

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

    if (!tracking_enabled_) {
      status_color = cv::Scalar(128, 128, 128);  // Gray
      status_text = "IDLE";
      mode_text = "Press [A] to start";
    } else if (tracking_state_ == TrackingState::TRACKING) {
      status_color = cv::Scalar(0, 165, 255);  // Orange
      status_text = "TRACKING";
      mode_text = "KCF";
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
    std::string hint = tracking_enabled_ ? "[A] STOP" : "[A] START";
    cv::Scalar hint_color = tracking_enabled_ ? cv::Scalar(0, 100, 255) : cv::Scalar(0, 255, 0);
    cv::putText(frame, hint, cv::Point(w - 100, 25),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, hint_color, 2);

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

  // OpenCV / MobileNet-SSD
  cv::dnn::Net net_;
  cv::VideoCapture cap_;

  // Tracker
  cv::Ptr<cv::Tracker> tracker_;
  cv::Rect tracked_bbox_;
  bool tracker_initialized_ = false;
  TrackingState tracking_state_ = TrackingState::DETECTING;

  // Joystick state
  bool last_tracking_button_state_ = false;
  bool tracking_enabled_ = false;  // Aボタンで追跡ON/OFF

  // Tracking lost detection
  int tracking_lost_count_ = 0;
  int verification_counter_ = 0;
  int detection_miss_count_ = 0;

  // Async detection state for DETECTING mode
  bool detection_in_progress_ = false;

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
  double velocity_feedforward_gain_;

  // Tracking parameters
  int tracking_button_index_;
  int lost_threshold_;
  int verification_interval_;
  double iou_threshold_;
  int detection_miss_threshold_;
  bool show_window_;

  // Model parameters
  int detection_skip_frames_;
  int ssd_person_class_id_ = 15;  // person class in VOC dataset
  int detection_frame_counter_ = 0;
  double kcf_scale_ = 0.5;  // KCFトラッカー用のスケール（0.5 = 160x120）

  // Scan parameters
  bool scan_enabled_;
  double scan_pan_velocity_;
  double scan_tilt_velocity_;
  double scan_pan_min_;
  double scan_pan_max_;
  double scan_tilt_min_;
  double scan_tilt_max_;
  double scan_tilt_step_;

  // Scan state
  ScanDirection scan_direction_ = ScanDirection::RIGHT;
  double scan_target_tilt_ = 0.0;  // 現在のスキャンチルト目標

  // Auto-fire parameters
  bool auto_fire_enabled_;
  double auto_fire_threshold_;
  int auto_fire_cooldown_ms_;
  double auto_fire_center_tolerance_;
  double shooter_speed_rpm_;
  int shooter_spinup_ms_;
  int shooter_fire_duration_ms_;
  double shooter_speed_rad_s_;

  // Auto-fire state
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr shooter_pub_;
  FiringState firing_state_ = FiringState::IDLE;
  std::chrono::steady_clock::time_point firing_state_start_time_;
  std::chrono::steady_clock::time_point last_fire_time_;
  float last_detection_confidence_ = 0.0f;  // 最後に検出された信頼度

  // Current joint positions
  double current_pan_ = 0.0;
  double current_tilt_ = 0.0;

  // PI control state
  double pan_integral_error_ = 0.0;
  double tilt_integral_error_ = 0.0;

  // Velocity feedforward state (previous frame target position)
  int prev_body_center_x_ = -1;
  int prev_body_center_y_ = -1;

  // Async YOLO thread
  std::thread yolo_thread_;
  std::atomic<bool> yolo_thread_running_{false};
  std::mutex yolo_mutex_;
  std::condition_variable yolo_cv_;
  cv::Mat yolo_frame_;
  bool yolo_request_pending_ = false;

  // Async YOLO results
  std::mutex yolo_result_mutex_;
  DetectionResult yolo_async_result_;
  bool yolo_result_ready_ = false;
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
