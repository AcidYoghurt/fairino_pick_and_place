#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fairino_msg/msg/qr_msg.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/wechat_qrcode.hpp>


// =============================================================
// 工具函数：点排序（强鲁棒）
// supports: 任意旋转、角点乱序、不稳定检测框
// =============================================================
bool robustSortCorners(std::vector<cv::Point2f> &c)
{
    if (c.size() != 4) return false;

    // 求中心
    cv::Point2f center(0,0);
    for (auto &p : c) center += p;
    center *= 0.25f;

    // 结果
    std::vector<cv::Point2f> sorted(4);

    for (auto &p : c)
    {
        if (p.x < center.x && p.y < center.y) sorted[0] = p;  // 左上
        else if (p.x > center.x && p.y < center.y) sorted[1] = p; // 右上
        else if (p.x > center.x && p.y > center.y) sorted[2] = p; // 右下
        else sorted[3] = p;  // 左下
    }
    c = sorted;
    return true;
}

class WeChatQRCodeNode : public rclcpp::Node {
public:
    WeChatQRCodeNode(const std::string& name)
        : Node(name)
    {
        std::string pkg = ament_index_cpp::get_package_share_directory("perception_layer");

        // WeChat QR 模型路径
        detector_ = cv::makePtr<cv::wechat_qrcode::WeChatQRCode>(
            pkg + "/config/wechatQR/detect.prototxt",
            pkg + "/config/wechatQR/detect.caffemodel",
            pkg + "/config/wechatQR/sr.prototxt",
            pkg + "/config/wechatQR/sr.caffemodel"
        );

        // 订阅相机图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "color_image_rect", 10,
            std::bind(&WeChatQRCodeNode::imageCallback, this, std::placeholders::_1));

        // 发布可视化图
        viz_pub_ = this->create_publisher<sensor_msgs::msg::Image>("qr/image", 10);

        // 发布角点+中心点
        qr_pub_ = this->create_publisher<fairino_msg::msg::QrMsg>("qr/qr_msg", 10);

        RCLCPP_INFO(this->get_logger(), "QR Stabilizer v5 started.");
    }

private:

// =============================================================
// 滤波器与状态
// =============================================================
std::deque<bool> detect_window_;                 // 滑动窗口（存最近 5 帧是否检测成功）
const int WINDOW_SIZE = 5;
const int WINDOW_PASS = 3;                      // ≥3 帧成功认为稳定

bool stable_ = false;

std::vector<cv::Point2f> filtered_corners_;      // EMA 平滑后的角点
std::vector<cv::Point2f> last_good_corners_;     // 最近一次的有效角点

float alpha_ = 0.4;  // EMA 权重，越小越稳（工业推荐 0.3~0.5）

// =============================================================
// 主回调函数
// =============================================================
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    // 1. 转 OpenCV 图像
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (...) {
        return;
    }
    cv::Mat frame = cv_ptr->image;
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    bool found = false;
    std::vector<cv::Point2f> corners;

    // =============================================================
    // STEP 1：使用 QRCodeDetector 检测角点
    // =============================================================
    cv::QRCodeDetector detect;
    std::vector<cv::Point> int_pts;
    found = detect.detect(gray, int_pts);

    if (found && int_pts.size() == 4)
    {
        corners.clear();
        for (auto &p : int_pts) corners.emplace_back((float)p.x, (float)p.y);

        // 强鲁棒排序
        robustSortCorners(corners);
    }

    // =============================================================
    // STEP 2：使用 WeChatQRCode 解码
    // =============================================================
    std::vector<cv::Mat> points;
    std::vector<std::string> results = detector_->detectAndDecode(gray, points);

    std::string decoded = results.empty() ? "" : results[0];

    if (decoded.empty())
        found = false;   // 解码失败 → 不算有效

    // =============================================================
    // STEP 3：滑动窗口统计
    // =============================================================
    detect_window_.push_back(found);
    if ((int)detect_window_.size() > WINDOW_SIZE)
        detect_window_.pop_back();

    int pass = 0;
    for (bool f : detect_window_) if (f) pass++;

    stable_ = (pass >= WINDOW_PASS);   // 5 帧中 ≥3 帧成功 → 稳定

    // =============================================================
    // STEP 4：处理稳定逻辑（关键）
    // =============================================================
    if (!found)
    {
        // 没检测到：如果之前稳定过 → 使用旧角点
        if (stable_ && !filtered_corners_.empty())
        {
            corners = filtered_corners_;
        }
        else
        {
            publishLost(frame, msg->header);
            return;
        }
    }

    // 保存最近有效角点
    last_good_corners_ = corners;

    // =============================================================
    // STEP 5：EMA 平滑滤波
    // =============================================================
    if (filtered_corners_.empty())
    {
        filtered_corners_ = corners;  // 初始化
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            filtered_corners_[i].x = alpha_ * corners[i].x + (1 - alpha_) * filtered_corners_[i].x;
            filtered_corners_[i].y = alpha_ * corners[i].y + (1 - alpha_) * filtered_corners_[i].y;
        }
    }

    // =============================================================
    // STEP 6：计算中心点
    // =============================================================
    cv::Point2f center(0,0);
    for (auto &p : filtered_corners_) center += p;
    center *= 0.25f;

    // =============================================================
    // STEP 7：绘图
    // =============================================================
    cv::Mat viz = frame.clone();
    for (auto &p : filtered_corners_)
        cv::circle(viz, p, 5, cv::Scalar(0,255,0), -1);

    std::vector<cv::Point> poly;
    for (auto &p : filtered_corners_) poly.emplace_back((int)p.x, (int)p.y);
    cv::polylines(viz, poly, true, cv::Scalar(0,255,0), 2);

    auto out = cv_bridge::CvImage(msg->header, "bgr8", viz).toImageMsg();
    viz_pub_->publish(*out);

    // =============================================================
    // STEP 8：发布消息 QrMsg
    // =============================================================
    fairino_msg::msg::QrMsg qr;
    qr.header = msg->header;
    qr.item_id.data = decoded;
    qr.car_id.data = "";

    qr.center_point.x = center.x;
    qr.center_point.y = center.y;
    qr.center_point.z = 0;

    for (auto &p : filtered_corners_) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0;
        qr.corners.push_back(pt);
    }

    qr_pub_->publish(qr);
}


// LOST 情况的可视化（画红框）
void publishLost(const cv::Mat &frame, const std_msgs::msg::Header &header)
{
    cv::Mat lost = frame.clone();
    cv::putText(lost, "QR LOST", cv::Point(50,50), 1, 2.0, cv::Scalar(0,0,255), 3);

    auto out = cv_bridge::CvImage(header, "bgr8", lost).toImageMsg();
    viz_pub_->publish(*out);

    // 发布空 qr_msg
    fairino_msg::msg::QrMsg qr;
    qr.header = header;
    qr_pub_->publish(qr);
}


private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viz_pub_;
    rclcpp::Publisher<fairino_msg::msg::QrMsg>::SharedPtr qr_pub_;

    cv::Ptr<cv::wechat_qrcode::WeChatQRCode> detector_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeChatQRCodeNode>("qr_stabilizer_v5"));
    rclcpp::shutdown();
    return 0;
}
