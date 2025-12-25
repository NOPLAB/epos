#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "chikurin_control/body_tracker/params.hpp"
#include "chikurin_control/body_tracker/person_detector.hpp"
#include "chikurin_control/body_tracker/target_tracker.hpp"
#include "chikurin_control/body_tracker/pan_tilt_controller.hpp"
#include "chikurin_control/body_tracker/scan_controller.hpp"
#include "chikurin_control/body_tracker/firing_controller.hpp"
#include "chikurin_control/body_tracker/tracker_ui.hpp"

using namespace body_tracker;

class BodyTrackerNode : public rclcpp::Node
{
public:
  BodyTrackerNode() : Node("body_tracker")
  {
    declareParameters();
    loadParameters();
    initializeComponents();
    setupROS();

    RCLCPP_INFO(get_logger(), "Body tracker node started (pan: Kp=%.4f Ki=%.5f, tilt: Kp=%.4f Ki=%.5f)",
                pan_tilt_params_.pan_kp, pan_tilt_params_.pan_ki,
                pan_tilt_params_.tilt_kp, pan_tilt_params_.tilt_ki);
  }

  ~BodyTrackerNode()
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void declareParameters()
  {
    // PanTilt parameters
    declare_parameter("pan_kp", 0.002);
    declare_parameter("pan_ki", 0.0001);
    declare_parameter("tilt_kp", 0.002);
    declare_parameter("tilt_ki", 0.0001);
    declare_parameter("pan_limit", M_PI);
    declare_parameter("tilt_limit", M_PI / 2);
    declare_parameter("pan_base_threshold", 0.7);
    declare_parameter("base_angular_gain", 0.5);
    declare_parameter("velocity_feedforward_gain", 0.005);

    // Detection parameters
    declare_parameter("confidence_threshold", 0.5);
    declare_parameter("nms_threshold", 0.4);
    declare_parameter("detection_skip_frames", 3);

    // Tracking parameters
    declare_parameter("tracking_button_index", 0);
    declare_parameter("lost_threshold", 150);
    declare_parameter("verification_interval", 30);
    declare_parameter("iou_threshold", 0.3);
    declare_parameter("detection_miss_threshold", 3);

    // Scan parameters
    declare_parameter("scan_enabled", true);
    declare_parameter("scan_pan_velocity", 0.5);
    declare_parameter("scan_tilt_velocity", 0.3);
    declare_parameter("pan_counts_per_revolution", 6600000.0);
    declare_parameter("tilt_counts_per_revolution", 6600000.0);
    declare_parameter("scan_pan_min_counts", -1650000);
    declare_parameter("scan_pan_max_counts", 1650000);
    declare_parameter("scan_tilt_min_counts", -500000);
    declare_parameter("scan_tilt_max_counts", 300000);
    declare_parameter("scan_tilt_step_counts", 200000);

    // Auto-fire parameters
    declare_parameter("auto_fire_enabled", false);
    declare_parameter("auto_fire_threshold", 0.7);
    declare_parameter("auto_fire_cooldown_ms", 2000);
    declare_parameter("auto_fire_center_tolerance", 0.15);
    declare_parameter("shooter_speed_rpm", 10000.0);
    declare_parameter("shooter_spinup_ms", 500);
    declare_parameter("shooter_fire_duration_ms", 300);

    // Camera parameters
    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("use_camera_device", true);
    declare_parameter("camera_device_id", 0);
    declare_parameter("show_window", true);
  }

  void loadParameters()
  {
    // PanTilt
    pan_tilt_params_.pan_kp = get_parameter("pan_kp").as_double();
    pan_tilt_params_.pan_ki = get_parameter("pan_ki").as_double();
    pan_tilt_params_.tilt_kp = get_parameter("tilt_kp").as_double();
    pan_tilt_params_.tilt_ki = get_parameter("tilt_ki").as_double();
    pan_tilt_params_.pan_limit = get_parameter("pan_limit").as_double();
    pan_tilt_params_.tilt_limit = get_parameter("tilt_limit").as_double();
    pan_tilt_params_.pan_base_threshold = get_parameter("pan_base_threshold").as_double();
    pan_tilt_params_.base_angular_gain = get_parameter("base_angular_gain").as_double();
    pan_tilt_params_.velocity_feedforward_gain = get_parameter("velocity_feedforward_gain").as_double();

    // Detection
    detection_params_.confidence_threshold = get_parameter("confidence_threshold").as_double();
    detection_params_.nms_threshold = get_parameter("nms_threshold").as_double();
    detection_params_.detection_skip_frames = get_parameter("detection_skip_frames").as_int();

    // Tracking
    tracking_params_.tracking_button_index = get_parameter("tracking_button_index").as_int();
    tracking_params_.lost_threshold = get_parameter("lost_threshold").as_int();
    tracking_params_.verification_interval = get_parameter("verification_interval").as_int();
    tracking_params_.iou_threshold = get_parameter("iou_threshold").as_double();
    tracking_params_.detection_miss_threshold = get_parameter("detection_miss_threshold").as_int();

    // Scan
    scan_params_.enabled = get_parameter("scan_enabled").as_bool();
    scan_params_.pan_velocity = get_parameter("scan_pan_velocity").as_double();
    scan_params_.tilt_velocity = get_parameter("scan_tilt_velocity").as_double();
    scan_params_.pan_counts_per_revolution = get_parameter("pan_counts_per_revolution").as_double();
    scan_params_.tilt_counts_per_revolution = get_parameter("tilt_counts_per_revolution").as_double();
    scan_params_.pan_min_counts = get_parameter("scan_pan_min_counts").as_int();
    scan_params_.pan_max_counts = get_parameter("scan_pan_max_counts").as_int();
    scan_params_.tilt_min_counts = get_parameter("scan_tilt_min_counts").as_int();
    scan_params_.tilt_max_counts = get_parameter("scan_tilt_max_counts").as_int();
    scan_params_.tilt_step_counts = get_parameter("scan_tilt_step_counts").as_int();

    // Auto-fire
    auto_fire_params_.enabled = get_parameter("auto_fire_enabled").as_bool();
    auto_fire_params_.threshold = get_parameter("auto_fire_threshold").as_double();
    auto_fire_params_.cooldown_ms = get_parameter("auto_fire_cooldown_ms").as_int();
    auto_fire_params_.center_tolerance = get_parameter("auto_fire_center_tolerance").as_double();
    auto_fire_params_.shooter_speed_rpm = get_parameter("shooter_speed_rpm").as_double();
    auto_fire_params_.spinup_ms = get_parameter("shooter_spinup_ms").as_int();
    auto_fire_params_.fire_duration_ms = get_parameter("shooter_fire_duration_ms").as_int();

    // Camera
    camera_params_.use_device = get_parameter("use_camera_device").as_bool();
    camera_params_.device_id = get_parameter("camera_device_id").as_int();
    camera_params_.image_topic = get_parameter("image_topic").as_string();
    camera_params_.show_window = get_parameter("show_window").as_bool();
  }

  void initializeComponents()
  {
    // Create components
    detector_ = std::make_unique<PersonDetector>(detection_params_);
    tracker_ = std::make_unique<TargetTracker>(tracking_params_);
    pan_tilt_controller_ = std::make_unique<PanTiltController>(pan_tilt_params_);
    scan_controller_ = std::make_unique<ScanController>(scan_params_);
    firing_controller_ = std::make_unique<FiringController>(auto_fire_params_);
    ui_ = std::make_unique<TrackerUI>(pan_tilt_params_);

    // Load detection model
    std::string pkg_share = ament_index_cpp::get_package_share_directory("shooter_control");
    std::string prototxt_path = pkg_share + "/models/MobileNetSSD_deploy.prototxt";
    std::string caffemodel_path = pkg_share + "/models/MobileNetSSD_deploy.caffemodel";

    RCLCPP_INFO(get_logger(), "Loading MobileNet-SSD model from: %s", pkg_share.c_str());
    if (!detector_->loadModel(prototxt_path, caffemodel_path)) {
      RCLCPP_ERROR(get_logger(), "Failed to load MobileNet-SSD model");
      throw std::runtime_error("Failed to load model");
    }
    RCLCPP_INFO(get_logger(), "MobileNet-SSD model loaded successfully");
  }

  void setupROS()
  {
    // Publishers
    pan_tilt_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pan_tilt_controller/commands", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive_controller/cmd_vel_unstamped", 10);
    shooter_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/shooter_controller/commands", 10);

    // Set firing controller callbacks
    firing_controller_->setCallbacks(
      [this](double vel) { publishShooterVelocity(vel); },
      [this]() { callTriggerService(); }
    );

    // Subscribers
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&BodyTrackerNode::jointStateCallback, this, std::placeholders::_1));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&BodyTrackerNode::joyCallback, this, std::placeholders::_1));

    // Auto-fire service client
    if (auto_fire_params_.enabled) {
      trigger_client_ = create_client<std_srvs::srv::Trigger>("/servo_trigger_node/trigger");
      RCLCPP_INFO(get_logger(), "Auto-fire enabled, waiting for trigger service...");
      if (trigger_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_INFO(get_logger(), "Servo trigger service is available");
      } else {
        RCLCPP_WARN(get_logger(), "Servo trigger service not available");
      }
    }

    // Camera setup
    if (camera_params_.use_device) {
      cap_.open(camera_params_.device_id);
      if (!cap_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera device %d", camera_params_.device_id);
        throw std::runtime_error("Failed to open camera");
      }
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

      timer_ = create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&BodyTrackerNode::cameraCallback, this));
    } else {
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        camera_params_.image_topic, 10,
        std::bind(&BodyTrackerNode::imageCallback, this, std::placeholders::_1));
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "pan_joint" && i < msg->position.size()) {
        current_pan_ = msg->position[i];
        current_pan_counts_ = ScanController::radiansToEncoderCounts(
          current_pan_, scan_params_.pan_counts_per_revolution);
      } else if (msg->name[i] == "tilt_joint" && i < msg->position.size()) {
        current_tilt_ = msg->position[i];
        current_tilt_counts_ = ScanController::radiansToEncoderCounts(
          current_tilt_, scan_params_.tilt_counts_per_revolution);
      }
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (static_cast<size_t>(tracking_params_.tracking_button_index) >= msg->buttons.size()) {
      return;
    }

    bool current_button = msg->buttons[tracking_params_.tracking_button_index] != 0;
    if (current_button && !last_button_state_) {
      toggleTracking();
    }
    last_button_state_ = current_button;
  }

  void toggleTracking()
  {
    tracking_enabled_ = !tracking_enabled_;

    if (tracking_enabled_) {
      tracker_->setState(TrackingState::DETECTING);
      scan_controller_->reset();
      RCLCPP_INFO(get_logger(), "Tracking ENABLED - scanning for target");
    } else {
      stopTracking();
      RCLCPP_INFO(get_logger(), "Tracking DISABLED");
    }
  }

  void stopTracking()
  {
    tracker_->reset();
    pan_tilt_controller_->reset();
    firing_controller_->stop();
    tracking_enabled_ = false;
  }

  void returnToDetecting()
  {
    tracker_->reset();
    pan_tilt_controller_->reset();
    scan_controller_->reset();
    detection_in_progress_ = false;
    RCLCPP_INFO(get_logger(), "Returning to DETECTING mode");
  }

  void cameraCallback()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN(get_logger(), "Failed to read frame from camera");
      return;
    }
    processFrame(frame);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    processFrame(cv_ptr->image);
  }

  void processFrame(cv::Mat& frame)
  {
    firing_controller_->update();

    int width = frame.cols;
    int height = frame.rows;
    int frame_center_x = width / 2;
    int frame_center_y = height / 2;

    cv::Rect target_box;
    bool target_valid = false;
    float target_confidence = 0.0f;
    DetectionResult detection_result;

    if (tracker_->getState() == TrackingState::TRACKING) {
      processTrackingMode(frame, target_box, target_valid, detection_result);
    } else if (tracking_enabled_) {
      processDetectingMode(frame, target_box, target_valid, detection_result);
    }

    // Compute control outputs
    double pan_velocity = 0.0;
    double tilt_velocity = 0.0;
    ControlOutput control_output;

    if (!tracking_enabled_) {
      pan_tilt_controller_->reset();
    } else if (target_valid) {
      int body_center_x = target_box.x + target_box.width / 2;
      int body_center_y = target_box.y + target_box.height / 2;

      control_output = pan_tilt_controller_->compute(
        body_center_x, body_center_y,
        frame_center_x, frame_center_y,
        current_pan_, current_tilt_);

      pan_tilt_controller_->updatePrevCenter(body_center_x, body_center_y);

      pan_velocity = control_output.pan_velocity;
      tilt_velocity = control_output.tilt_velocity;

      // Try auto-fire in tracking mode
      if (tracker_->getState() == TrackingState::TRACKING) {
        firing_controller_->tryFire(last_detection_confidence_,
                                    control_output.pan_error, control_output.tilt_error);
      }
    } else {
      pan_tilt_controller_->decay();

      // Scan in detecting mode
      if (tracking_enabled_ && tracker_->getState() == TrackingState::DETECTING) {
        auto scan_vel = scan_controller_->calculate(current_pan_counts_, current_tilt_counts_);
        pan_velocity = scan_vel.pan;
        tilt_velocity = scan_vel.tilt;
      }
    }

    // Publish commands
    publishPanTiltCommand(pan_velocity, tilt_velocity);
    publishBaseVelocity(target_valid ? control_output.base_angular_velocity : 0.0);

    // Draw UI
    if (camera_params_.show_window) {
      UIState ui_state;
      ui_state.tracking_enabled = tracking_enabled_;
      ui_state.tracking_state = tracker_->getState();
      ui_state.target_valid = target_valid;
      ui_state.target_box = target_box;
      ui_state.target_confidence = target_confidence;
      ui_state.pan_error = control_output.pan_error;
      ui_state.tilt_error = control_output.tilt_error;
      ui_state.pan_p_term = control_output.pan_p_term;
      ui_state.pan_i_term = control_output.pan_i_term;
      ui_state.tilt_p_term = control_output.tilt_p_term;
      ui_state.tilt_i_term = control_output.tilt_i_term;
      ui_state.current_pan = current_pan_;
      ui_state.current_tilt = current_tilt_;
      ui_state.target_pan = current_pan_ + control_output.pan_velocity;
      ui_state.target_tilt = current_tilt_ + control_output.tilt_velocity;
      ui_state.base_angular_velocity = control_output.base_angular_velocity;
      ui_state.num_detections = detection_result.count();
      ui_state.tracking_lost_count = tracker_->getLostCount();
      ui_state.lost_threshold = tracking_params_.lost_threshold;

      // Draw other detections in detecting mode
      if (tracking_enabled_ && tracker_->getState() == TrackingState::DETECTING) {
        for (int idx : detection_result.indices) {
          if (detection_result.boxes[idx] != target_box) {
            cv::rectangle(frame, detection_result.boxes[idx], cv::Scalar(100, 100, 100), 1);
          }
        }
      }

      ui_->draw(frame, ui_state);
      ui_->show(frame);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "Control: pan_vel=%.4f tilt_vel=%.4f target_valid=%d state=%s",
      pan_velocity, tilt_velocity, target_valid,
      tracker_->getState() == TrackingState::TRACKING ? "TRACKING" : "DETECTING");
  }

  void processTrackingMode(cv::Mat& frame, cv::Rect& target_box, bool& target_valid,
                           DetectionResult& detection_result)
  {
    bool track_success = tracker_->update(frame);

    // Request verification periodically
    tracker_->incrementVerificationCounter();
    if (tracker_->needsVerification()) {
      tracker_->resetVerificationCounter();
      detector_->requestAsyncDetection(frame);
    }

    // Check async detection result
    DetectionResult async_result;
    if (detector_->getAsyncResult(async_result)) {
      detection_result = async_result;

      if (async_result.empty()) {
        tracker_->incrementDetectionMissCount();
        if (tracker_->hasExceededMissThreshold()) {
          RCLCPP_WARN(get_logger(), "Detection verification failed, returning to detecting mode");
          returnToDetecting();
          return;
        }
      } else {
        // Find best matching detection
        double best_iou = 0.0;
        cv::Rect best_match;
        float best_confidence = 0.0f;

        for (int idx : async_result.indices) {
          double iou = PersonDetector::calculateIoU(tracker_->getBoundingBox(), async_result.boxes[idx]);
          if (iou > best_iou) {
            best_iou = iou;
            best_match = async_result.boxes[idx];
            best_confidence = async_result.confidences[idx];
          }
        }

        if (best_iou > tracker_->getIoUThreshold()) {
          tracker_->resetDetectionMissCount();
          last_detection_confidence_ = best_confidence;
          tracker_->reinit(frame, best_match);
        } else {
          tracker_->incrementDetectionMissCount();
          if (tracker_->hasExceededMissThreshold()) {
            RCLCPP_WARN(get_logger(), "Tracking target mismatch, returning to detecting mode");
            returnToDetecting();
            return;
          }
        }
      }
    }

    if (track_success) {
      tracker_->resetLostCount();
      target_box = tracker_->getBoundingBox();
      target_valid = true;
    } else {
      tracker_->incrementLostCount();
      if (tracker_->isLost()) {
        RCLCPP_WARN(get_logger(), "KCF tracking lost, returning to detecting mode");
        returnToDetecting();
      } else {
        target_box = tracker_->getBoundingBox();
        target_valid = true;
      }
    }
  }

  void processDetectingMode(cv::Mat& frame, cv::Rect& target_box, bool& target_valid,
                            DetectionResult& detection_result)
  {
    detection_frame_counter_++;
    if (detection_frame_counter_ >= detection_params_.detection_skip_frames) {
      detection_frame_counter_ = 0;
      if (!detection_in_progress_) {
        detector_->requestAsyncDetection(frame);
        detection_in_progress_ = true;
      }
    }

    DetectionResult async_result;
    if (detector_->getAsyncResult(async_result)) {
      detection_in_progress_ = false;
      detection_result = async_result;

      if (!async_result.empty()) {
        cv::Rect selected = detector_->selectBestTarget(async_result, frame.cols, frame.rows);
        if (tracker_->init(frame, selected)) {
          target_box = selected;
          target_valid = true;
          RCLCPP_INFO(get_logger(), "Person detected, auto-starting tracking");
        }
      }
    }
  }

  void publishPanTiltCommand(double pan_vel, double tilt_vel)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {pan_vel, tilt_vel};
    pan_tilt_pub_->publish(msg);
  }

  void publishBaseVelocity(double angular_vel)
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = angular_vel;
    cmd_vel_pub_->publish(msg);
  }

  void publishShooterVelocity(double velocity)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {velocity};
    shooter_pub_->publish(msg);
  }

  void callTriggerService()
  {
    if (trigger_client_ && trigger_client_->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      trigger_client_->async_send_request(request);
      RCLCPP_INFO(get_logger(), "Servo trigger service called");
    } else {
      RCLCPP_ERROR(get_logger(), "Servo trigger service not ready");
    }
  }

  // Parameter structures
  PanTiltParams pan_tilt_params_;
  DetectionParams detection_params_;
  TrackingParams tracking_params_;
  ScanParams scan_params_;
  AutoFireParams auto_fire_params_;
  CameraParams camera_params_;

  // Components
  std::unique_ptr<PersonDetector> detector_;
  std::unique_ptr<TargetTracker> tracker_;
  std::unique_ptr<PanTiltController> pan_tilt_controller_;
  std::unique_ptr<ScanController> scan_controller_;
  std::unique_ptr<FiringController> firing_controller_;
  std::unique_ptr<TrackerUI> ui_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pan_tilt_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr shooter_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Camera
  cv::VideoCapture cap_;

  // State
  bool tracking_enabled_ = false;
  bool last_button_state_ = false;
  bool detection_in_progress_ = false;
  int detection_frame_counter_ = 0;
  float last_detection_confidence_ = 0.0f;

  // Current joint positions
  double current_pan_ = 0.0;
  double current_tilt_ = 0.0;
  int current_pan_counts_ = 0;
  int current_tilt_counts_ = 0;
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
