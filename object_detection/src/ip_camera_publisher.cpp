/// \file
/// \brief This node publishes IP camera streams to a ROS topic
///
/// PARAMETER:
///     rtsp_url (std::string): RTSP URL of the IP camera
/// PUBLISHES:
///     /camera/image (sensor_msgs::msg::Image): camera images

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ament_index_cpp/get_package_prefix.hpp"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

/// \brief A node for publishing IP camera streams
class IPCameraPublisher : public rclcpp::Node
{
public:
  IPCameraPublisher()
  : Node("ip_camera_publisher")
  {
    // Initializes the publisher, reads RTSP URL from configuration, and starts capturing and
    // publishing images
    auto package_prefix_directory = ament_index_cpp::get_package_prefix("object_detection");
    auto config_path = package_prefix_directory + "/lib/object_detection/config.json";
    std::ifstream config_file(config_path);
    nlohmann::json config_json;
    config_file >> config_json;
    std::string rtsp_url = config_json["rtsp_url"];

    declare_parameter("rtsp_url", rtsp_url);
    rtsp_url_ = get_parameter("rtsp_url").get_parameter_value().get<std::string>();

    if (rtsp_url_.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("IPCameraPublisher"), "RTSP URL is empty");
      return;
    }

    camera_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

    // Open the video stream using the RTSP URL
    cv::VideoCapture cap(rtsp_url_);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(rclcpp::get_logger("IPCameraPublisher"), "Could not open the video stream");
      return;
    }

    cv::namedWindow("Camera Stream", cv::WINDOW_AUTOSIZE);

    // Main loop to capture and publish the video frames
    while (rclcpp::ok()) {
      cv::Mat frame;
      cap >> frame;
      if (frame.empty()) {
        break;
      }

      cv::imshow("Camera Stream", frame);
      cv::waitKey(1);

      // Convert the captured frame to a ROS image message and publish
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      camera_pub_->publish(*msg);

      // Allow ROS to process incoming messages
      rclcpp::spin_some(get_node_base_interface());
    }

    cv::destroyAllWindows();
  }

private:
  // Declare private variables
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  std::string rtsp_url_;
};

/// \brief The main function
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IPCameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
