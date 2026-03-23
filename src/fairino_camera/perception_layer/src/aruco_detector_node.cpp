// aruco_detector_node_30fps_optimized.cpp
// 核心原则：30 FPS 实时视频流 + 10 Hz ArUco 检测解耦
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <deque>
#include <mutex>

class ArucoDetectorNode : public rclcpp::Node {
public:
    ArucoDetectorNode() : Node("aruco_detector_node") {
        declare_parameter<double>("marker_length", 0.05);
        declare_parameter<double>("smoothing_alpha", 0.5);
        declare_parameter<int>("stability_window_size", 3);
        declare_parameter<double>("stability_position_threshold", 0.05);
        declare_parameter<double>("detect_interval", 0.1); 

        get_parameter("marker_length", marker_length_);
        get_parameter("smoothing_alpha", alpha_);
        get_parameter("stability_window_size", stability_window_size_);
        get_parameter("stability_position_threshold", stability_pos_thres_);
        get_parameter("detect_interval", detect_interval_);

        auto qos = rclcpp::SensorDataQoS();
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/color/camera_info", 10,
            std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1)
        );


  
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "camera/color/image_raw", qos,
            std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));
        binary_pub_ = create_publisher<sensor_msgs::msg::Image>("aruco/binary", 10);
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("aruco/image", 10);
        pose_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("aruco/pose", 10);
        id_pub_    = create_publisher<std_msgs::msg::Int32MultiArray>("aruco/detected_markers", 10);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
        detector_params_ = cv::aruco::DetectorParameters::create();

        detector_params_->adaptiveThreshWinSizeMin = 5;
        detector_params_->adaptiveThreshWinSizeMax = 45;
        detector_params_->adaptiveThreshWinSizeStep = 10;

        // 背光下非常重要
        detector_params_->adaptiveThreshConstant = 3;  // 你现在 7 偏大

        // 放宽候选轮廓
        detector_params_->minMarkerPerimeterRate = 0.02;
        detector_params_->maxMarkerPerimeterRate = 4.0;

        // 容忍畸变 & 不完整边
        detector_params_->errorCorrectionRate = 0.8;

        // 减少木纹干扰
        detector_params_->polygonalApproxAccuracyRate = 0.04;

        // 背光下角点容易贴边
        detector_params_->minCornerDistanceRate = 0.02;




        last_detect_time_ = now();
        RCLCPP_INFO(get_logger(), "Aruco detector (30FPS optimized) started");
    }

private:
    //结构体，用于存储检测结果
    struct DetectionResult {
        bool valid = false;
        int id = -1;
        std::vector<cv::Point2f> corners;
        geometry_msgs::msg::PoseStamped pose;
        bool is_stable = false;
        cv::Vec3d rvec; 
        cv::Vec3d tvec; 
    };

    //相机内参回调函数，用于获取相机内参
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (camera_info_received_) return;

        camera_matrix_ = (cv::Mat1d(3,3) <<
            msg->k[0], 0, msg->k[2],
            0, msg->k[4], msg->k[5],
            0, 0, 1);

        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        for (size_t i = 0; i < std::min<size_t>(5, msg->d.size()); ++i) {
            dist_coeffs_.at<double>(0, i) = msg->d[i];
        }

        camera_info_received_ = true;

        RCLCPP_INFO(get_logger(),
            "CameraInfo OK fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
    }

    //视频流回调函数，用于对视频流进行检测和绘制可视化处理
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!camera_info_received_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try { cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); }
        catch (...) { return; }

        cv::Mat annotated = cv_ptr->image.clone();

        // 快路径逻辑
        {
            std::lock_guard<std::mutex> lk(detect_mtx_);
            if (last_detection_.valid) {
                std::vector<std::vector<cv::Point2f>> draw_corners = {last_detection_.corners};
                std::vector<int> draw_ids = {last_detection_.id};

                // 1. 绘制边框和 ID
                cv::aruco::drawDetectedMarkers(annotated, draw_corners, draw_ids);

                // 2. 绘制 XYZ 坐标轴 (新增)
                // 参数: 图像, 内参, 畸变, 旋转向量, 位移向量, 轴长 (米)
                cv::drawFrameAxes(annotated, camera_matrix_, dist_coeffs_, 
                                last_detection_.rvec, last_detection_.tvec, 
                                marker_length_ * 1.5f, 3); // 3是线宽

                // 3. 绘制稳定性文本
                cv::Scalar color = last_detection_.is_stable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                std::string text = last_detection_.is_stable ? "STABLE" : "DETECTED";
                cv::putText(annotated, text, last_detection_.corners[0], 
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

            } else {
                cv::putText(annotated, "Searching for ArUco...", cv::Point(50, 50), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            }
        }

        image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg());

        // 慢路径检测触发
        if ((now() - last_detect_time_).seconds() >= detect_interval_) {
            last_detect_time_ = now();
            detectOnce(cv_ptr->image, msg->header);
        }
    }

    void detectOnce(const cv::Mat &image, const std_msgs::msg::Header &header) {
        if (camera_matrix_.empty() || dist_coeffs_.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "相机相关变量出现问题，跳过检测");
            return;
        }
        
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // 1. Gamma 校正（非常关键，对背光效果立竿见影）
        cv::Mat gamma_corrected;
        double gamma = 1.5;  // 1.3 ~ 1.8 之间调，背光越强越大
        cv::Mat lut(1, 256, CV_8U);
        for (int i = 0; i < 256; ++i) {
            lut.at<uchar>(i) = cv::saturate_cast<uchar>(
                std::pow(i / 255.0, 1.0 / gamma) * 255.0
            );
        }
        cv::LUT(gray, lut, gamma_corrected);

        // 2. CLAHE（tile 要比你现在小）
        cv::Mat enhanced;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
            2.0,              // clipLimit 不要太大
            cv::Size(4, 4)    // 小 tile，防止木纹被拉出来
        );
        clahe->apply(gamma_corrected, enhanced);


        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(enhanced, dictionary_, corners, ids, detector_params_);

        if (ids.empty()) {
            resetStability();
            return;
        }

        // 寻找最接近画面中心的 Marker
        int best = -1;
        double min_dist = 1e9;
        cv::Point2f img_center(camera_matrix_.at<double>(0,2), camera_matrix_.at<double>(1,2));
        for (size_t i = 0; i < ids.size(); ++i) {
            cv::Point2f c(0,0);
            for (auto &p : corners[i]) c += p;
            c *= 0.25f;
            double d = cv::norm(c - img_center);
            if (d < min_dist) { min_dist = d; best = i; }
        }
        if (best < 0) return;
        // 显式创建 target_corners 变量，避免传递临时初始化列表
        std::vector<cv::Vec3d> rvecs, tvecs;
        std::vector<std::vector<cv::Point2f>> target_corners;
        target_corners.push_back(corners[best]);

        // 将显式变量传递给 OpenCV 函数
        cv::aruco::estimatePoseSingleMarkers(
            target_corners, 
            marker_length_, 
            camera_matrix_, 
            dist_coeffs_, 
            rvecs, 
            tvecs
        );


        //构造原始pose消息，用于后续稳定性过滤
        geometry_msgs::msg::PoseStamped raw;
        raw.header = header;
        raw.pose.position.x = tvecs[0][0];
        raw.pose.position.y = tvecs[0][1];
        raw.pose.position.z = tvecs[0][2];

        cv::Mat R;
        cv::Rodrigues(rvecs[0], R);
        tf2::Matrix3x3 tf_R(
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
        tf2::Quaternion q;
        tf_R.getRotation(q);
        raw.pose.orientation = tf2::toMsg(q);

        geometry_msgs::msg::PoseStamped out;
        if (!checkStabilityAndFilter(ids[best], raw, out)) return;

        // 更新最后一次成功检测的结果
        std::lock_guard<std::mutex> lk(detect_mtx_);
        last_detection_.valid = true;
        last_detection_.id = ids[best];
        last_detection_.corners = corners[best];
        last_detection_.pose = out;
        last_detection_.rvec = rvecs[0];
        last_detection_.tvec = tvecs[0];
        last_detection_.is_stable = true;

        // 发布 Pose 和 ID
        pose_pub_->publish(out);
        std_msgs::msg::Int32MultiArray id_msg;
        id_msg.data.push_back(ids[best]);
        id_pub_->publish(id_msg);
    }

    bool checkStabilityAndFilter(int id, const geometry_msgs::msg::PoseStamped &in,
                                geometry_msgs::msg::PoseStamped &out) {
        std::lock_guard<std::mutex> lock(filter_mtx_);


        //如果当前id更新，清除历史记录
        if (current_tracking_id_ != id) {
            pose_buffer_.clear();
            current_tracking_id_ = id;
            has_last_stable_pose_ = false;
        }

        pose_buffer_.push_back(in);
        if (pose_buffer_.size() > (size_t)stability_window_size_)
            pose_buffer_.pop_front();
        if (pose_buffer_.size() < (size_t)stability_window_size_) return false;

        const auto &ref = pose_buffer_.back().pose.position;
        for (auto &p : pose_buffer_) {
            double d = std::hypot(p.pose.position.x - ref.x,
                                  p.pose.position.y - ref.y,
                                  p.pose.position.z - ref.z);
            if (d > stability_pos_thres_) return false;
        }

        if (!has_last_stable_pose_) {
            last_stable_pose_ = in;
            has_last_stable_pose_ = true;
            out = in;
            return true;
        }

        geometry_msgs::msg::PoseStamped sm = in;
        sm.pose.position.x = alpha_ * in.pose.position.x + (1 - alpha_) * last_stable_pose_.pose.position.x;
        sm.pose.position.y = alpha_ * in.pose.position.y + (1 - alpha_) * last_stable_pose_.pose.position.y;
        sm.pose.position.z = alpha_ * in.pose.position.z + (1 - alpha_) * last_stable_pose_.pose.position.z;

        tf2::Quaternion qc, qp;
        tf2::fromMsg(in.pose.orientation, qc);
        tf2::fromMsg(last_stable_pose_.pose.orientation, qp);
        if (qc.dot(qp) < 0) qc = tf2::Quaternion(-qc.x(), -qc.y(), -qc.z(), -qc.w());
        sm.pose.orientation = tf2::toMsg(qc.slerp(qp, 1.0 - alpha_));

        last_stable_pose_ = sm;
        out = sm;
        return true;
    }

    void resetStability() {
        std::lock_guard<std::mutex> lock(filter_mtx_);
        pose_buffer_.clear();
        current_tracking_id_ = -1;
        has_last_stable_pose_ = false;
        std::lock_guard<std::mutex> lk(detect_mtx_);
        last_detection_.valid = false;
    }

    // -------- Members --------
    bool camera_info_received_ = false;
    cv::Mat camera_matrix_, dist_coeffs_;

    double marker_length_, alpha_, stability_pos_thres_, detect_interval_;
    int stability_window_size_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr id_pub_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    rclcpp::Time last_detect_time_;
    std::mutex detect_mtx_;
    DetectionResult last_detection_;

    std::mutex filter_mtx_;
    int current_tracking_id_ = -1;
    std::deque<geometry_msgs::msg::PoseStamped> pose_buffer_;
    geometry_msgs::msg::PoseStamped last_stable_pose_;
    bool has_last_stable_pose_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
