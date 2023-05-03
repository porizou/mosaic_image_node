#include <iostream>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"

using namespace std::chrono_literals;

class MosaicNode : public rclcpp::Node
{
public:
  MosaicNode() : Node("mosaic_node")
  {
    this->declare_parameter("mosaic_size", 10);
    int mosaic_size = this->get_parameter("mosaic_size").as_int();
    
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw",
      10,
      std::bind(&MosaicNode::image_callback, this, std::placeholders::_1));

    mosaic_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("output_mosaic_image", 10);
    mosaic_size_ = mosaic_size;
    face_cascade_.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat mosaic_image = apply_mosaic(cv_ptr->image, mosaic_size_);

    sensor_msgs::msg::Image::SharedPtr mosaic_msg = cv_bridge::CvImage(msg->header, "bgr8", mosaic_image).toImageMsg();
    mosaic_publisher_->publish(*mosaic_msg);
  }

  cv::Mat apply_mosaic(const cv::Mat& input_image, int mosaic_size)
  {
    cv::Mat mosaic_image = input_image.clone();
    std::vector<cv::Rect> faces;

    // Detect faces
    face_cascade_.detectMultiScale(mosaic_image, faces, 1.1, 3, 0, cv::Size(30, 30));

    // Apply mosaic to detected faces
    for (const auto& face : faces)
    {
      cv::Mat face_roi = mosaic_image(face);
      cv::Mat mosaic_face;
      cv::resize(face_roi, mosaic_face, cv::Size(), 1.0 / mosaic_size, 1.0 / mosaic_size, cv::INTER_LINEAR);
      cv::resize(mosaic_face, mosaic_face, face_roi.size(), 0, 0, cv::INTER_NEAREST);
      mosaic_face.copyTo(face_roi);
    }

    return mosaic_image;
  }

  // Private member variables
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mosaic_publisher_;
  int mosaic_size_;
  cv::CascadeClassifier face_cascade_;
};  // class MosaicNode

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MosaicNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

