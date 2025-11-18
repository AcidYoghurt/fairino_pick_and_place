#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nlohmann/json.hpp>
#include <fairino_msg/msg/qr_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/wechat_qrcode.hpp>


class WeChatQRCodeNode : public rclcpp::Node {
public:
    WeChatQRCodeNode(const std::string& name) : Node(name) {
        std::string package_path = ament_index_cpp::get_package_share_directory("item_recognition");

        // 参数
        this->declare_parameter("test", false);
        this->get_parameter("test", is_test);

        // 模型路径文件
        detect_prototxt = package_path+"/config/wechatQR/detect.prototxt";
        detect_caffemodel = package_path+"/config/wechatQR/detect.caffemodel";
        sr_prototxt = package_path+"/config/wechatQR/sr.prototxt";
        sr_caffemodel = package_path+"/config/wechatQR/sr.caffemodel";
        if (detect_prototxt.empty() || detect_caffemodel.empty() || sr_prototxt.empty() || sr_caffemodel.empty()) {
            RCLCPP_FATAL(this->get_logger(), "未提供所有模型文件路径，节点无法初始化！");
            return;
        }

        // 初始化微信二维码检测器
        try {
            detector_ = cv::makePtr<cv::wechat_qrcode::WeChatQRCode>(
                detect_prototxt,
                detect_caffemodel,
                sr_prototxt,
                sr_caffemodel
            );
        } catch (const cv::Exception& e) {
            RCLCPP_FATAL(this->get_logger(), "初始化微信二维码检测器失败: %s", e.what());
            return;
        }

        // 创建订阅者和发布者
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "color_image_rect", 10, std::bind(&WeChatQRCodeNode::image_callback, this, std::placeholders::_1));

        annotated_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("qr/image", 10);

        qr_msg_publisher_ = this->create_publisher<fairino_msg::msg::QrMsg>("qr/qr_msg",10);

        qr_point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("qr/center_point",10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
            return;
        }

        cv::Mat annotated_img = cv_ptr->image.clone();

        // 使用微信二维码检测器进行检测和解码
        std::vector<cv::Mat> points;
        std::vector<std::string> res = detector_->detectAndDecode(cv_ptr->image, points);

        // 遍历所有识别到的二维码
        for (size_t i = 0; i < res.size(); ++i)
        {
            // 查看是否识别到正确的二维码
            try
            {
                // RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"接收到二维码数据：%s",res[i].c_str());
                nlohmann::json json = nlohmann::json::parse(res[i]);    // 解析json数据

                // 获取角点
                const cv::Mat& current_qr_mat = points[i];
                std::vector<cv::Point2f> current_qr;
                if (current_qr_mat.total() == 8 && current_qr_mat.type() == 5)
                {
                    current_qr.assign((cv::Point2f*)current_qr_mat.data, (cv::Point2f*)current_qr_mat.data + 4);
                } else
                {
                    RCLCPP_WARN(this->get_logger(), "二维码角点数据格式异常");
                    continue;
                }

                // 绘制角点
                std::vector<cv::Point> current_qr_int;
                for (const auto& point : current_qr) {
                    current_qr_int.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
                    cv::circle(annotated_img, point, 5, cv::Scalar(0, 0, 255), -1);
                }

                // 绘制连接线
                cv::polylines(annotated_img, current_qr_int, true, cv::Scalar(0, 255, 0), 2);

                // 获取中心点
                std::vector<cv::Point2f> point_vector;
                cv::Point2f center_point(0, 0);
                for (const auto& point : current_qr) {
                    point_vector.push_back(point);
                    center_point += point;
                }
                center_point.x /= current_qr.size();
                center_point.y /= current_qr.size();

                // 绘制中心点
                cv::circle(annotated_img, center_point, 7, cv::Scalar(255, 0, 0), -1);
                cv::Point center_text_pos = cv::Point(static_cast<int>(center_point.x + 10), static_cast<int>(center_point.y + 5));

                // 绘制识别到的二维码数据
                if (!res[i].empty()) {
                    cv::Point text_pos = cv::Point(static_cast<int>(current_qr[0].x), static_cast<int>(current_qr[0].y - 10));
                    cv::putText(annotated_img, res[i], text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
                }
                if (json.contains("item_id"))
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"检测到货物");
                    cv::putText(annotated_img, "物品ID："+json["item_id"].get<std::string>(), center_text_pos, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
                    // 发布二维码信息
                    fairino_msg::msg::QrMsg qrMsg;
                    qrMsg.header = msg->header;
                    qrMsg.item_id.data = json["item_id"].get<std::string>();
                    qrMsg.car_id.data = "";
                    qrMsg.center_point.x = center_point.x;
                    qrMsg.center_point.y = center_point.y;
                    qr_msg_publisher_->publish(qrMsg);
                }
                else if (json.contains("car_id"))
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"检测到小车");
                    cv::putText(annotated_img, "小车ID："+json["car_id"].get<std::string>(), center_text_pos, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
                    // 发布二维码信息
                    fairino_msg::msg::QrMsg qrMsg;
                    qrMsg.header = msg->header;
                    qrMsg.item_id.data = "";
                    qrMsg.car_id.data = json["car_id"].get<std::string>();
                    qrMsg.header = msg->header;
                    qrMsg.center_point.x = center_point.x;
                    qrMsg.center_point.y = center_point.y;
                    qr_msg_publisher_->publish(qrMsg);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),"json格式不正确！请检查二维码json格式！");
                }
            } catch (nlohmann::detail::exception& e)
            {
                if (is_test)
                {
                    const cv::Mat& current_qr_mat = points[i];
                    std::vector<cv::Point2f> current_qr;
                    if (current_qr_mat.total() == 8 && current_qr_mat.type() == 5)
                    {
                        current_qr.assign((cv::Point2f*)current_qr_mat.data, (cv::Point2f*)current_qr_mat.data + 4);
                    } else
                    {
                        RCLCPP_WARN(this->get_logger(), "二维码角点数据格式异常");
                        continue;
                    }

                    // 绘制角点
                    std::vector<cv::Point> current_qr_int;
                    for (const auto& point : current_qr) {
                        current_qr_int.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
                        cv::circle(annotated_img, point, 5, cv::Scalar(0, 0, 255), -1);
                    }

                    // 绘制连接线
                    cv::polylines(annotated_img, current_qr_int, true, cv::Scalar(0, 255, 0), 2);

                    // 获取中心点
                    cv::Point2f center_point(0, 0);
                    for (const auto& point : current_qr) {
                        center_point += point;
                    }
                    center_point.x /= current_qr.size();
                    center_point.y /= current_qr.size();

                    // 绘制中心点
                    cv::circle(annotated_img, center_point, 7, cv::Scalar(255, 0, 0), -1);
                    cv::Point center_text_pos = cv::Point(static_cast<int>(center_point.x + 10), static_cast<int>(center_point.y + 5));
                    cv::putText(annotated_img, res[i], center_text_pos, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

                    // 绘制识别到的二维码数据
                    if (!res[i].empty()) {
                        cv::Point text_pos = cv::Point(static_cast<int>(current_qr[0].x), static_cast<int>(current_qr[0].y - 10));
                        cv::putText(annotated_img, res[i], text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
                    }

                    // 发布中心点
                    geometry_msgs::msg::PointStamped qrPoint;
                    qrPoint.header.stamp = msg->header.stamp;
                    qrPoint.header.frame_id = "camera_color_frame";
                    qrPoint.point.x = center_point.x;
                    qrPoint.point.y = center_point.y;
                    qr_point_publisher_->publish(qrPoint);
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000,"暂未识别到有效二维码：%s",e.what());
                }
            }
        }

        // 发布带标注的图像
        cv_bridge::CvImage annotated_cv_img;
        annotated_cv_img.header = msg->header;
        annotated_cv_img.encoding = "bgr8";
        annotated_cv_img.image = annotated_img;
        annotated_image_publisher_->publish(*annotated_cv_img.toImageMsg());
    }

    // 参数
    bool is_test;

    // 微信扫码模型文件
    std::string detect_prototxt;
    std::string detect_caffemodel;
    std::string sr_prototxt;
    std::string sr_caffemodel;

    // ROS 2 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_publisher_;
    rclcpp::Publisher<fairino_msg::msg::QrMsg>::SharedPtr qr_msg_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr qr_point_publisher_;
    cv::Ptr<cv::wechat_qrcode::WeChatQRCode> detector_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WeChatQRCodeNode>("wechat_qrcode_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}