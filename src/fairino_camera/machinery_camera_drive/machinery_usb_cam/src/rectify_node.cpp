#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <memory>
#include <opencv2/calib3d.hpp>


class RectifyNode : public rclcpp::Node
{
public:
  RectifyNode() : Node("rectify_node")
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&RectifyNode::image_callback, this, std::placeholders::_1));

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::QoS(10).reliable().durability_volatile(),
      std::bind(&RectifyNode::info_callback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_rect", 10);

    RCLCPP_INFO(this->get_logger(), "矫正节点启动。");
  }

private:
  void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    distortion_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F, (void*)msg->d.data()).clone();
    camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();

    if (!distortion_coeffs_.empty() && !camera_matrix_.empty()) {
        RCLCPP_INFO_ONCE(this->get_logger(), "相机内参已接收");
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (distortion_coeffs_.empty() || camera_matrix_.empty()) {
        RCLCPP_WARN_ONCE(this->get_logger(), "正在等待相机内参");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 格式
        // 目标编码指定为 "bgr8" 以确保能够显示颜色
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge 出错: %s", e.what());
        return;
    }

    cv::Mat rectified_image;
    cv::undistort(cv_ptr->image, rectified_image, camera_matrix_, distortion_coeffs_);

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = msg->header.frame_id;

    cv_bridge::CvImage rectified_cv_image(header, "bgr8", rectified_image);
    image_pub_->publish(*rectified_cv_image.toImageMsg());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RectifyNode>());
  rclcpp::shutdown();
  return 0;
}