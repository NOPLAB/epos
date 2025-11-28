#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

class FaceTrackerNode : public rclcpp::Node
{
public:
  FaceTrackerNode() : Node("face_tracker")
  {
    // Parameters
    declare_parameter("kp", 0.002);
    declare_parameter("ki", 0.0001);
    declare_parameter("max_angular_velocity", 1.5);
    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("cascade_file", "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");
    declare_parameter("use_camera_device", true);
    declare_parameter("camera_device_id", 0);

    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
    use_camera_device_ = get_parameter("use_camera_device").as_bool();
    camera_device_id_ = get_parameter("camera_device_id").as_int();

    // Load cascade classifier
    std::string cascade_file = get_parameter("cascade_file").as_string();
    if (!face_cascade_.load(cascade_file)) {
      RCLCPP_ERROR(get_logger(), "Failed to load cascade file: %s", cascade_file.c_str());
      throw std::runtime_error("Failed to load cascade file");
    }

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
        std::bind(&FaceTrackerNode::camera_callback, this));
    } else {
      // Subscribe to ROS image topic
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("image_topic").as_string(), 10,
        std::bind(&FaceTrackerNode::image_callback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(), "Face tracker node started (Kp=%.4f, Ki=%.4f)", kp_, ki_);
  }

  ~FaceTrackerNode()
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
    // Convert to grayscale for face detection
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);

    // Detect faces
    std::vector<cv::Rect> faces;
    face_cascade_.detectMultiScale(gray, faces, 1.1, 3, 0, cv::Size(80, 80));

    double angular_velocity = 0.0;
    double error = 0.0;
    double p_term = 0.0;
    double i_term = 0.0;
    int frame_center_x = frame.cols / 2;
    int frame_center_y = frame.rows / 2;
    int face_center_x = frame_center_x;
    bool face_detected = false;

    if (!faces.empty()) {
      face_detected = true;

      // Use the largest face
      cv::Rect largest_face = *std::max_element(faces.begin(), faces.end(),
        [](const cv::Rect& a, const cv::Rect& b) {
          return a.area() < b.area();
        });

      // Calculate face center
      face_center_x = largest_face.x + largest_face.width / 2;
      int face_center_y = largest_face.y + largest_face.height / 2;

      // Calculate error (normalized to -1.0 ~ 1.0)
      error = static_cast<double>(frame_center_x - face_center_x) / frame_center_x;

      // PI control
      integral_error_ += error;

      // Anti-windup
      double max_integral = max_angular_velocity_ / ki_;
      integral_error_ = std::clamp(integral_error_, -max_integral, max_integral);

      p_term = kp_ * error;
      i_term = ki_ * integral_error_;
      angular_velocity = p_term + i_term;
      angular_velocity = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);

      // Draw face rectangle
      cv::rectangle(frame, largest_face, cv::Scalar(0, 255, 0), 2);

      // Draw face center crosshair
      cv::drawMarker(frame, cv::Point(face_center_x, face_center_y),
        cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);

      // Draw line from center to face
      cv::line(frame, cv::Point(frame_center_x, frame_center_y),
        cv::Point(face_center_x, face_center_y), cv::Scalar(255, 255, 0), 2);
    } else {
      // No face detected - slowly reset integral
      integral_error_ *= 0.95;
    }

    // Publish velocity command
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = angular_velocity;
    cmd_vel_pub_->publish(msg);

    // === Draw UI ===
    draw_ui(frame, face_detected, error, p_term, i_term, angular_velocity);

    cv::imshow("Face Tracker", frame);
    cv::waitKey(1);
  }

  void draw_ui(cv::Mat& frame, bool face_detected, double error,
               double p_term, double i_term, double angular_velocity)
  {
    int w = frame.cols;
    int h = frame.rows;

    // Draw center line
    cv::line(frame, cv::Point(w / 2, 0), cv::Point(w / 2, h),
      cv::Scalar(100, 100, 100), 1);

    // Status panel background
    cv::rectangle(frame, cv::Point(10, 10), cv::Point(250, 160),
      cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(frame, cv::Point(10, 10), cv::Point(250, 160),
      cv::Scalar(100, 100, 100), 1);

    // Status text
    cv::Scalar status_color = face_detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    std::string status_text = face_detected ? "TRACKING" : "SEARCHING";
    cv::putText(frame, status_text, cv::Point(20, 35),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2);

    // PI values
    char buf[64];
    snprintf(buf, sizeof(buf), "Error: %+.3f", error);
    cv::putText(frame, buf, cv::Point(20, 60),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    snprintf(buf, sizeof(buf), "P: %+.4f", p_term);
    cv::putText(frame, buf, cv::Point(20, 80),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 200, 200), 1);

    snprintf(buf, sizeof(buf), "I: %+.4f", i_term);
    cv::putText(frame, buf, cv::Point(20, 100),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 255), 1);

    snprintf(buf, sizeof(buf), "Cmd: %+.3f rad/s", angular_velocity);
    cv::putText(frame, buf, cv::Point(20, 120),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

    // Gains
    snprintf(buf, sizeof(buf), "Kp=%.4f Ki=%.5f", kp_, ki_);
    cv::putText(frame, buf, cv::Point(20, 150),
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

  // OpenCV
  cv::CascadeClassifier face_cascade_;
  cv::VideoCapture cap_;

  // Parameters
  double kp_;
  double ki_;
  double max_angular_velocity_;
  bool use_camera_device_;
  int camera_device_id_;

  // PI control state
  double integral_error_ = 0.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FaceTrackerNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("face_tracker"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
