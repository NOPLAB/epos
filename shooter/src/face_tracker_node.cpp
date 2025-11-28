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
    int frame_center_x = frame.cols / 2;

    if (!faces.empty()) {
      // Use the largest face
      cv::Rect largest_face = *std::max_element(faces.begin(), faces.end(),
        [](const cv::Rect& a, const cv::Rect& b) {
          return a.area() < b.area();
        });

      // Calculate face center
      int face_center_x = largest_face.x + largest_face.width / 2;

      // Calculate error (normalized to -1.0 ~ 1.0)
      double error = static_cast<double>(frame_center_x - face_center_x) / frame_center_x;

      // PI control
      integral_error_ += error;

      // Anti-windup
      double max_integral = max_angular_velocity_ / ki_;
      integral_error_ = std::clamp(integral_error_, -max_integral, max_integral);

      angular_velocity = kp_ * error + ki_ * integral_error_;
      angular_velocity = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);

      RCLCPP_DEBUG(get_logger(), "Face at x=%d, error=%.3f, cmd=%.3f",
        face_center_x, error, angular_velocity);

      // Draw rectangle on face (for debugging with imshow)
      cv::rectangle(frame, largest_face, cv::Scalar(0, 255, 0), 2);
    } else {
      // No face detected - slowly reset integral
      integral_error_ *= 0.95;
    }

    // Publish velocity command
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = angular_velocity;
    cmd_vel_pub_->publish(msg);

    // Show frame for debugging (optional)
    cv::imshow("Face Tracker", frame);
    cv::waitKey(1);
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
