#include "yolo_object_detection/yolo_object_detection.hpp"


namespace yolo_object_detection
{

YoloObjectDetection::YoloObjectDetection()
: Node("yolo_object_detection_node")
{
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

}
