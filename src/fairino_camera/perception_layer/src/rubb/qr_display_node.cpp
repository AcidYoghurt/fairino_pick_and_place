#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp> // 用于 cv::imshow
#include <opencv2/imgproc.hpp> 

class QRDisplayNode : public rclcpp::Node
{
public:
    QRDisplayNode() : Node("qr_display_node")
    {
        // 创建订阅者，订阅 wechat_qr_detection 发布的带标注图像
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "qr/image", 10, std::bind(&QRDisplayNode::image_callback, this, std::placeholders::_1));
        
        // 创建 OpenCV 窗口
        cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "QRDisplayNode 启动，订阅话题: /qr/image");
    }

    ~QRDisplayNode()
    {
        cv::destroyWindow(WINDOW_NAME);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // 尝试将 ROS 2 Image 消息转换为 OpenCV Mat
            // wechat_qr_detection 发布的是 bgr8 编码
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
            return;
        }

        // 实时显示图像
        cv::imshow(WINDOW_NAME, cv_ptr->image);
        
        // 必须调用 waitKey 才能让 OpenCV 窗口正确刷新
        cv::waitKey(1); 
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    const std::string WINDOW_NAME = "QR Code Detection Result";
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QRDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}