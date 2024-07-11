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
  bool enable_gpu = declare_parameter("enable_gpu", false);
  fs::path model_path = declare_parameter("model_path", fs::path());
  fs::path model_file = model_path / declare_parameter("model_file", std::string());
//  fs::path class_file = model_path / declare_parameter("class", std::string());

  if (!fs::exists(model_file)) {
    RCLCPP_ERROR(get_logger(), "Load model failed");
    rclcpp::shutdown();
  }

  // Initialize inference here. Not ideal, just quickly make it runnable.
  // Note that in this example the classes are hard-coded and 'classes.txt' is a place holder.
  inference_ = yolo::Inference(model_file.string(), cv::Size(640, 480), "classes.txt", enable_gpu);

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
          cv::Size text_size = cv::getTextSize(class_string, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
          cv::Rect text_box(box.x, box.y - 40, text_size.width + 10, text_size.height + 20);

          cv::rectangle(cv_image, text_box, color, cv::FILLED);
          cv::putText(cv_image, class_string, cv::Point(box.x + 5, box.y - 10),
            cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
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

//  bool YoloObjectDetection::load_classes(fs::path class_file)
//  {
//    std::ifstream input_file(class_file);
//    if (!input_file.is_open()) {
//      RCLCPP_ERROR(get_logger(), "Unable to open class file");
//      return false;
//    } else {
//      std::string line;
//      while (std::getline(input_file, line)) {
//        class_list_.push_back(line);
//      }
//      return true;
//    }
//  }

//  void YoloObjectDetection::load_net(fs::path model_file)
//  {
//    net_ = cv::dnn::readNet(model_file);
//    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
//      RCLCPP_INFO(get_logger(), "CUDA is available. Attempy to use CUDA");
//      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
//      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
//    } else {
//      RCLCPP_INFO(get_logger(), "No CUDA-enabled devices found. Running on CPU");
//      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
//      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
//    }
//  }

//  cv::Mat YoloObjectDetection::format_yolov5(const cv::Mat & source)
//  {
//    int col = source.cols;
//    int row = source.rows;
//    int max = std::max(col, row);
//    cv::Mat result = cv::Mat::zeros(max, max, CV_8UC3);
//    source.copyTo(result(cv::Rect(0, 0, col, row)));

//    return result;
//  }

//  void YoloObjectDetection::detect(cv::Mat & image, std::vector<Detection> & output)
//  {
//    auto input_image = format_yolov5(image);

//    cv::Mat blob;
//    cv::dnn::blobFromImage(
//      input_image, blob, 1.0 / 255.0, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
//    net_.setInput(blob);

//    std::vector<cv::Mat> outputs;
//    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

//    float x_factor = input_image.cols / INPUT_WIDTH;
//    float y_factor = input_image.rows / INPUT_HEIGHT;

//    float * data = (float *)outputs[0].data;

//    std::vector<int> class_ids;
//    std::vector<float> confidences;
//    std::vector<cv::Rect> boxes;

//    for (int i = 0; i < OUTPUT_ROWS; ++i) {
//      float confidence = data[4];
//      if (confidence >= CONFIDENCE_THRESHOLD) {
//        float * classes_scores = data + 5;
//        cv::Mat scores(1, class_list_.size(), CV_32FC1, classes_scores);
//        cv::Point class_id;
//        double max_class_score;
//        cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
//        if (max_class_score > SCORE_THRESHOLD) {
//          confidences.push_back(confidence);
//          class_ids.push_back(class_id.x);

//          float x = data[0];
//          float y = data[1];
//          float w = data[2];
//          float h = data[3];
//          int left = int((x - 0.5 * w) * x_factor);
//          int top = int((y - 0.5 * h) * y_factor);
//          int width = int(w * x_factor);
//          int height = int(h * y_factor);
//          boxes.push_back(cv::Rect(left, top, width, height));
//        }
//      }
//      data += CLASS_DIMENSIONS;
//    }

//    std::vector<int> nms_result;
//    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
//    for (size_t i = 0; i < nms_result.size(); ++i) {
//      int idx = nms_result[i];
//      Detection result;
//      result.class_id = class_ids[idx];
//      result.confidence = confidences[idx];
//      result.box = boxes[idx];
//      output.push_back(result);
//    }
//  }

} // namespace yolo_object_detection
