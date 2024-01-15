// C++ header
#include <fstream>

// local header
#include "yolo_object_detection/yolo_object_detection.hpp"


namespace yolo_object_detection
{

YoloObjectDetection::YoloObjectDetection()
: Node("yolo_object_detection_node")
{
  fs::path model_path = declare_parameter("model_path", fs::path());
  fs::path model_file = model_path / declare_parameter("model", std::string());
  fs::path class_file = model_path / declare_parameter("class", std::string());

  if (!load_classes(class_file)) {
    RCLCPP_ERROR(get_logger(), "Load class list failed");
    rclcpp::shutdown();
  }

  load_net(model_file);

  rclcpp::QoS qos(10);
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "kitti/camera/color/left/image_raw", qos, std::bind(
      &YoloObjectDetection::img_callback, this, std::placeholders::_1));
}

void YoloObjectDetection::img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  img_buff_.push(msg);
}

bool YoloObjectDetection::load_classes(fs::path class_file)
{
  std::ifstream input_file(class_file);
  if (!input_file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Unable to open class file");
    return false;
  } else {
    std::string line;
    while (std::getline(input_file, line)) {
      class_list_.push_back(line);
    }
    return true;
  }
}

void YoloObjectDetection::load_net(fs::path model_file)
{
  net_ = cv::dnn::readNet(model_file);
  if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
    RCLCPP_INFO(get_logger(), "CUDA is available. Attempy to use CUDA");
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
  } else {
    RCLCPP_INFO(get_logger(), "No CUDA-enabled devices found. Running on CPU");
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }
}

}
