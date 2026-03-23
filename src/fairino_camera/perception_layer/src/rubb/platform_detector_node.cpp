#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cmath> // 包含 M_PI 常量
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// #include "tf2/LinearMath/Quaternion.h" // 如果只发布PointStamped，可以不需要tf2依赖

/**
 * @brief ROS 2 节点：订阅图像，检测白色平台上的黑色矩形，并发布中心点坐标。
 */
class PlatformDetectorNode : public rclcpp::Node
{
public:
    PlatformDetectorNode() : Node("platform_detector_node")
    {
        // 1. 订阅器：订阅相机发布的图像话题
        // 保持与二维码节点一致的订阅话题
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/color/image_raw", 10,
            std::bind(&PlatformDetectorNode::image_callback, this, std::placeholders::_1));

        // 2. 发布器：发布检测到的矩形中心点 (PointStamped)
        // PointStamped 只包含 (x, y, z) 坐标，不包含姿态
        center_point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "platform/center_point", 10);

        // 3. 发布器：发布带标注的图像（用于调试）
        annotated_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "platform/image", 10);

        RCLCPP_INFO(this->get_logger(), "Platform Detector Node Initialized. Subscribing to 'color_image_rect'...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_point_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_publisher_;

    /**
     * @brief 图像处理核心函数：检测黑色矩形
     * @param input_image 原始图像 (cv::Mat)
     * @param center_x 找到的矩形中心点X (像素)
     * @param center_y 找到的矩形中心点Y (像素)
     * @param angle_deg 找到的矩形旋转角度 (度) - 尽管您只需要中心点，但角度有助于可视化
     * @return true 如果成功找到矩形，否则为 false
     */
    bool process_image(const cv::Mat& input_image, 
                       double& center_x, double& center_y, double& angle_deg, 
                       cv::Mat& output_annotated_img)
    {
        if (input_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Input image is empty.");
            return false;
        }

        output_annotated_img = input_image.clone(); // 用于绘制调试信息

        // 1. 预处理 (灰度化和高斯模糊)
        cv::Mat gray, blur, thresh;
        cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);
        // 使用较大的内核尺寸有助于平滑白色平台上的细微瑕疵
        cv::GaussianBlur(gray, blur, cv::Size(7, 7), 0); 

        // 2. 阈值化 (白色背景上的黑色物体 -> THRESH_BINARY_INV | OTSU)
        // 重点：使用反向二值化，使得黑色线条区域为白色前景(255)
        cv::threshold(blur, thresh, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

        // 3. 轮廓查找
        std::vector<std::vector<cv::Point>> contours;
        // 查找最外层轮廓
        cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double min_area = 1000.0; // 根据您的实际平台尺寸调整最小面积阈值

        for (const auto& contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < min_area) continue;

            // 形状近似
            double perimeter = cv::arcLength(contour, true);
            std::vector<cv::Point> approx;
            // 0.04 是近似精度，用于识别接近完美的直线矩形
            cv::approxPolyDP(contour, approx, 0.04 * perimeter, true);

            // 4. 筛选：寻找 4 个顶点的轮廓 (矩形)
            if (approx.size() == 4)
            {
                // 5. 获取最小外接旋转矩形（用于获取中心和角度）
                cv::RotatedRect rect = cv::minAreaRect(contour);
                cv::Point2f center = rect.center;
                angle_deg = rect.angle;

                center_x = center.x;
                center_y = center.y;

                // --- 绘制调试信息 ---
                cv::Point2f rect_points[4];
                rect.points(rect_points);
                std::vector<cv::Point> box_pts;
                for(int i = 0; i < 4; i++) {
                    box_pts.push_back(cv::Point(static_cast<int>(rect_points[i].x), static_cast<int>(rect_points[i].y)));
                }

                // 绘制找到的矩形
                cv::polylines(output_annotated_img, box_pts, true, cv::Scalar(0, 255, 0), 2);
                
                // 绘制中心点
                cv::circle(output_annotated_img, center, 7, cv::Scalar(255, 0, 255), -1);
                
                // 绘制文本
                std::string text = "Center: (" + std::to_string(static_cast<int>(center_x)) + ", " + std::to_string(static_cast<int>(center_y)) + ")";
                cv::putText(output_annotated_img, text, center, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);

                return true; // 找到目标矩形并返回
            }
        }

        return false; // 未找到目标矩形
    }

    /**
     * @brief 图像消息回调函数
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            // 将 ROS 图像消息转换为 OpenCV Mat
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
            return;
        }

        double px_x, px_y, angle;
        cv::Mat annotated_img;
        
        // 1. 处理图像
        bool success = process_image(cv_ptr->image, px_x, px_y, angle, annotated_img);

        // 2. 发布带标注的图像
        cv_bridge::CvImage annotated_cv_img;
        annotated_cv_img.header = msg->header;
        annotated_cv_img.encoding = "bgr8";
        annotated_cv_img.image = annotated_img;
        annotated_image_publisher_->publish(*annotated_cv_img.toImageMsg());

        // 3. 发布结果
        if (success)
        {
            // 将像素坐标转换为世界坐标系下的 PointStamped
            geometry_msgs::msg::PointStamped target_point;
            
            // **** TODO: 替换为实际的世界坐标转换 ****
            // 暂时使用像素坐标作为 x 和 y
            target_point.point.x = px_x; 
            target_point.point.y = px_y; 
            target_point.point.z = 0.0; // 如果只看2D平面，Z可以为0

            // 设置消息头
            target_point.header.stamp = this->now();
            target_point.header.frame_id = "camera_color_frame"; // 与您的QR码节点保持一致

            center_point_publisher_->publish(target_point);
            RCLCPP_INFO(this->get_logger(), 
                "Published Center Point (Pixel): (X=%.2f, Y=%.2f)", 
                px_x, px_y
            );
        } 
        else 
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Target platform rectangle not found.");
        }
    }
};

// 节点主入口函数
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // 启动节点
    rclcpp::spin(std::make_shared<PlatformDetectorNode>());
    rclcpp::shutdown();
    return 0;
}