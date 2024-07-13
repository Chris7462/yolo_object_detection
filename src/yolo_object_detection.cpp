// C++ header
//  #include <fstream>
#include <string>
#include <chrono>
#include <filesystem>

// OpenCV header
#include <opencv2/core.hpp>

//  #include <opencv2/core/cuda.hpp>

// ROS header
#include <cv_bridge/cv_bridge.h>

//  // local header
//  #include "yolo_object_detection/yolo_const.hpp"
#include "yolo_object_detection/yolo_object_detection.hpp"


namespace yolo_object_detection
{

namespace fs = std::filesystem;
using namespace std::chrono_literals;

YoloObjectDetection::YoloObjectDetection()
: Node("yolo_object_detection_node"), inference_()
{
  fs::path model_path = declare_parameter("model_path", fs::path());
  fs::path model_file = model_path / declare_parameter("model_file", std::string());
  // fs::path class_file = model_path / declare_parameter("class", std::string());

  if (!fs::exists(model_file)) {
    RCLCPP_ERROR(get_logger(), "Load model failed");
    rclcpp::shutdown();
  }

  // Initialize inference here. Not ideal, just quickly make it runnable.
  // Note that in this example the classes are hard-coded and 'classes.txt' is a place holder.
  inference_ = yolo::Inference(model_file.string(), cv::Size(640, 480), "classes.txt");

  rclcpp::QoS qos(10);
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "kitti/camera/color/left/image_raw", qos, std::bind(
      &YoloObjectDetection::img_callback, this, std::placeholders::_1));

  yolo_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "yolo_object_detection", qos);

  timer_ = this->create_wall_timer(
    25ms, std::bind(&YoloObjectDetection::timer_callback, this));
}

void YoloObjectDetection::img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  img_buff_.push(msg);
}

void YoloObjectDetection::timer_callback()
{
  if (!img_buff_.empty()) {
    rclcpp::Time current_time = rclcpp::Node::now();
    mtx_.lock();
    if ((current_time - rclcpp::Time(img_buff_.front()->header.stamp)).seconds() > 0.1) {
      // time sync has problem
      RCLCPP_WARN(get_logger(), "Timestamp unaligned, please check your IMAGE data.");
      img_buff_.pop();
      mtx_.unlock();
    } else {
      auto input_msg = img_buff_.front();
      img_buff_.pop();
      mtx_.unlock();

      try {
        cv::Mat cv_image = cv_bridge::toCvCopy(input_msg, "bgr8")->image;

        // Inference starts here...
        std::vector<yolo::Detection> detections = inference_.runInference(cv_image);

        // size_t detection_size = detections.size();
        // std::cout << "Number of detections:" << detection_size << std::endl;

        for (const auto & detection : detections) {
          auto box = detection.box;
          auto color = detection.color;
          // auto class_id = detection.class_id;

          // Detection box
          cv::rectangle(cv_image, box, color, 2);

          // Detection box text
          std::string class_string = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
          //  cv::Size text_size = cv::getTextSize(class_string, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
          //  cv::Rect text_box(box.x, box.y - 40, text_size.width + 10, text_size.height + 20);

          //  cv::rectangle(cv_image, text_box, color, cv::FILLED);
          //  cv::putText(cv_image, class_string, cv::Point(box.x + 5, box.y - 10),
          //    cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
          cv::rectangle(
            cv_image, cv::Point(box.x, box.y - 10.0),
            cv::Point(box.x + box.width, box.y), color, cv::FILLED);
          cv::putText(
            cv_image, class_string, cv::Point(box.x, box.y - 5.0),
            cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(0.0, 0.0, 0.0));
        }
        // Inference ends here...

        // Convert OpenCV image to ROS Image message
        auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();
        out_msg->header.frame_id = "cam2_link";
        out_msg->header.stamp = current_time;
        yolo_pub_->publish(*out_msg);

      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "CV_Bridge exception: %s", e.what());
      }
    }
  }
}

} // namespace yolo_object_detection
