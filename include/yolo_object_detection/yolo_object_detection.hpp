#pragma once

// C++ header
#include <atomic>
#include <filesystem>
#include <memory>
#include <queue>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <sensor_msgs/msg/image.hpp>

// local header
#include "yolo_inference/inference.hpp"

// OpenCV header
#include <opencv2/core.hpp>

namespace yolo_object_detection
{

const std::vector<cv::Scalar> COLORS = {
  cv::Scalar(255, 255, 0),
  cv::Scalar(0, 255, 0),
  cv::Scalar(0, 255, 255),
  cv::Scalar(255, 0, 0)};

namespace fs = std::filesystem;

class YoloObjectDetection : public rclcpp::Node
{
public:
  YoloObjectDetection();
  ~YoloObjectDetection();

private:
  bool initialize_parameters();
  void initialize_ros_components();

  void img_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void timer_callback();

  void publish_detection_result_image(
    const cv::Mat & result_image,
    const std_msgs::msg::Header & header);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr yolo_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_buff_;
  std::mutex mtx_;
  std::atomic<bool> processing_in_progress_;

  yolo::Inference inference_;

  fs::path model_path_;
  fs::path model_file_;

  int max_processing_queue_size_;
  double processing_frequency_;
};

} // namespace yolo_object_detection
