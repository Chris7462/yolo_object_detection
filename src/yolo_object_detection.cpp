// C++ header
#include <string>
#include <chrono>
#include <functional>
#include <exception>

// OpenCV header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// ROS header
#include <cv_bridge/cv_bridge.hpp>

// local header
#include "yolo_object_detection/yolo_object_detection.hpp"


namespace yolo_object_detection
{

YoloObjectDetection::YoloObjectDetection()
: Node("yolo_object_detection_node"),
  processing_in_progress_(false)
{
  if (!initialize_parameters()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize parameters");
    rclcpp::shutdown();
    return;
  }

  inference_ = yolo::Inference(model_file_.string(), cv::Size(640, 480), "");

  initialize_ros_components();

  RCLCPP_INFO(get_logger(),
    "YOLO object detection node initialized successfully with bounded queue (max: %d)",
    max_processing_queue_size_);
}

YoloObjectDetection::~YoloObjectDetection()
{
  RCLCPP_INFO(get_logger(), "YOLO object detection node shutting down");
}

bool YoloObjectDetection::initialize_parameters()
{
  try {
    model_path_ = fs::path(declare_parameter("model_path", ""));
    model_file_ = model_path_ / declare_parameter("model_file", "");

    if (model_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "Model file path is empty");
      return false;
    }

    if (!fs::exists(model_file_)) {
      RCLCPP_ERROR(get_logger(), "Model file does not exist: %s", model_file_.c_str());
      return false;
    }

    processing_frequency_ = declare_parameter<double>("processing_frequency", 50.0);
    if (processing_frequency_ <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid processing frequency: %.2f Hz", processing_frequency_);
      return false;
    }

    max_processing_queue_size_ = declare_parameter<int>("max_processing_queue_size", 3);
    if (max_processing_queue_size_ <= 0 || max_processing_queue_size_ > 10) {
      RCLCPP_ERROR(get_logger(), "Invalid max processing queue size: %d (should be 1-10)",
        max_processing_queue_size_);
      return false;
    }

    input_topic_ = declare_parameter("input_topic", std::string(""));
    if (input_topic_.empty()) {
      RCLCPP_ERROR(get_logger(),
        "input_topic is empty. This must be remapped by the launch file "
        "(e.g. input_topic:=/carla/hero/cam2/image) - refusing to start "
        "with an unspecified input source.");
      return false;
    }

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during parameter initialization: %s", e.what());
    return false;
  }
}

void YoloObjectDetection::initialize_ros_components()
{
  rclcpp::QoS image_qos(10);
  image_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  image_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  image_qos.history(rclcpp::HistoryPolicy::KeepLast);

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  img_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic_, image_qos,
    std::bind(&YoloObjectDetection::img_callback, this, std::placeholders::_1),
    sub_options
  );

  std::string output_topic = declare_parameter("output_topic", "yolo_object_detection");

  yolo_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, image_qos);

  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&YoloObjectDetection::timer_callback, this),
    callback_group_
  );

  RCLCPP_INFO(get_logger(), "ROS components initialized");
  RCLCPP_INFO(get_logger(), "Input: %s, Output: %s, Frequency: %.1f Hz",
    input_topic_.c_str(), output_topic.c_str(), processing_frequency_);
}

void YoloObjectDetection::img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    std::lock_guard<std::mutex> lock(mtx_);

    if (img_buff_.size() >= static_cast<size_t>(max_processing_queue_size_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Processing queue full, dropping oldest image (queue size: %ld)", img_buff_.size());
      img_buff_.pop();
    }

    img_buff_.push(msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in image callback: %s", e.what());
  }
}

void YoloObjectDetection::timer_callback()
{
  bool expected = false;
  if (!processing_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  sensor_msgs::msg::Image::SharedPtr msg;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (img_buff_.empty()) {
      processing_in_progress_.store(false);
      return;
    }
    msg = img_buff_.front();
    img_buff_.pop();
  }

  try {
    cv_bridge::CvImageConstPtr cv_ptr =
      cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

    if (!cv_ptr || cv_ptr->image.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty or invalid image");
      processing_in_progress_.store(false);
      return;
    }


    cv::Mat image_for_plot = cv_ptr->image.clone();

    std::vector<yolo::Detection> detections;

    try {
      detections = inference_.runInference(cv_ptr->image);
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Inference failed, skipping this frame");
      processing_in_progress_.store(false);
      return;
    }

    for (const auto & detection : detections) {
      auto box = detection.box;
      auto class_id = detection.class_id;
      auto color = COLORS[class_id % COLORS.size()];

      cv::rectangle(image_for_plot, box, color, 2);

      std::string class_string = detection.className + ' ' +
        std::to_string(detection.confidence).substr(0, 4);

      cv::rectangle(
        image_for_plot, cv::Point(box.x, box.y - 10.0),
        cv::Point(box.x + box.width, box.y), color, cv::FILLED);
      cv::putText(
        image_for_plot, class_string, cv::Point(box.x, box.y - 5.0),
        cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(0.0, 0.0, 0.0));
    }

    if (yolo_pub_->get_subscription_count() > 0) {
      publish_detection_result_image(image_for_plot, msg->header);
    }

  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during image processing: %s", e.what());
  }

  processing_in_progress_.store(false);
}

void YoloObjectDetection::publish_detection_result_image(
  const cv::Mat & result_image,
  const std_msgs::msg::Header & header)
{
  try {
    cv_bridge::CvImage cv_image;
    cv_image.header = header;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = result_image;

    auto output_msg = cv_image.toImageMsg();
    yolo_pub_->publish(*output_msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during result publishing: %s", e.what());
  }
}

} // namespace yolo_object_detection
