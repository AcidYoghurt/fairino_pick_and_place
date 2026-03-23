// collision_monitor_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <mutex>

class CollisionMonitorNode : public rclcpp::Node {
public:
    CollisionMonitorNode() : Node("collision_monitor_node") {
        // --- 参数 ---
        declare_parameter<double>("marker_length", 0.05); // 用于计算相对位置
        declare_parameter<double>("x_axis_grip_allowance_m", 0.02);
        declare_parameter<double>("depth_similarity_tolerance_m", 0.02);
        declare_parameter<double>("obstacle_in_front_margin_m", 0.02);
        declare_parameter<double>("x_axis_cargo_buffer_m", 0.01);
        declare_parameter<double>("y_axis_check_half_width_m", 0.01);

        get_parameter("marker_length", marker_length_);
        get_parameter("x_axis_grip_allowance_m", grip_allowance_width_m_);
        get_parameter("depth_similarity_tolerance_m", depth_similarity_tolerance_m_);
        get_parameter("obstacle_in_front_margin_m", obstacle_in_front_margin_m_);
        get_parameter("x_axis_cargo_buffer_m", x_axis_cargo_buffer_m_);
        get_parameter("y_axis_check_half_width_m", y_axis_check_half_width_m_);

        // --- 订阅 ---
        // 订阅深度图
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&CollisionMonitorNode::depthCallback, this, std::placeholders::_1));

        // 订阅 Aruco 节点发出的 Pose
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "aruco/pose", 10,
            std::bind(&CollisionMonitorNode::poseCallback, this, std::placeholders::_1));

        // 订阅相机内参 (用于 3D -> 2D 投影)
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/color/camera_info", rclcpp::QoS(1).transient_local(),
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                 if (!camera_info_received_) {
                     fx_ = msg->k[0]; fy_ = msg->k[4];
                     cx_ = msg->k[2]; cy_ = msg->k[5];
                     camera_info_received_ = true;
                     RCLCPP_INFO(get_logger(), "碰撞检测节点: 内参已接收");
                 }
            });

        // --- 发布 ---
        collision_status_pub_ = create_publisher<std_msgs::msg::Bool>("aruco/collision_risk", 10);
        collision_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("aruco/collision_marker", 10);

        RCLCPP_INFO(get_logger(), "碰撞检测节点已启动 (等待深度图与Pose)");
    }

private:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(depth_mtx_);
        try {
            // 简单的深度图转换逻辑 (保留你原有的处理)
            cv_bridge::CvImagePtr cv_ptr;
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                 latest_depth_ = cv_ptr->image.clone();
                 depth_is_32f_ = false;
            } else {
                 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
                 latest_depth_ = cv_ptr->image.clone();
                 depth_is_32f_ = true;
            }
        } catch (...) {}
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!camera_info_received_) return;

        std::lock_guard<std::mutex> lock(depth_mtx_);
        if (latest_depth_.empty()) return;

        // 1. 将 ROS Pose 转换为 OpenCV 矩阵 (R 和 T)
        // Position -> Tvec
        cv::Mat t_mat = (cv::Mat_<double>(3, 1) << 
            msg->pose.position.x, 
            msg->pose.position.y, 
            msg->pose.position.z);

        // Quaternion -> Rvec (或者直接用 Rotation Matrix)
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.orientation, q);
        tf2::Matrix3x3 tf_R(q);
        
        cv::Mat R_mat = (cv::Mat_<double>(3, 3) << 
            tf_R[0][0], tf_R[0][1], tf_R[0][2],
            tf_R[1][0], tf_R[1][1], tf_R[1][2],
            tf_R[2][0], tf_R[2][1], tf_R[2][2]);

        // 2. 执行碰撞检测
        bool collision = checkCollisionRisk(R_mat, t_mat);

        // 3. 发布状态
        std_msgs::msg::Bool status_msg;
        status_msg.data = collision;
        collision_status_pub_->publish(status_msg);

        // 4. 可视化
        if (collision) {
            publishCollisionMarker(msg->header, msg->pose);
        } else {
            clearCollisionMarker(msg->header);
        }
    }

    // 复用并微调原来的 checkCollisionRisk
    // 参数改为 R_mat 和 t_mat
    bool checkCollisionRisk(const cv::Mat& R_mat, const cv::Mat& t_mat) {
        double tag_depth = t_mat.at<double>(2,0);
        
        // 此处简化了逻辑：由于我们没有 Corners 2D 坐标了 (在另一个节点)，
        // 我们主要依赖 "Part 2: 夹爪 X 轴方向侧面检测 (3D 迭代)"
        // 如果需要 Part 1 (区域扫描)，你需要把 Tag 中心投射回像素坐标来估算范围。
        
        // 计算 Tag 中心像素坐标 (用于地面排除粗略估计)
        double u_center = cx_ + fx_ * (t_mat.at<double>(0,0) / tag_depth);
        double v_center = cy_ + fy_ * (t_mat.at<double>(1,0) / tag_depth);

        // 定义检测的物理距离范围 (米)
        double half_marker = marker_length_ * 0.5;
        double X_start_m = half_marker + x_axis_cargo_buffer_m_;
        double X_end_m = half_marker + grip_allowance_width_m_;
        double step_x_m = 0.01; 
        double step_y_m = 0.02; 
        double Y_half_range = y_axis_check_half_width_m_;

        for (double y_offset = -Y_half_range; y_offset <= Y_half_range; y_offset += step_y_m) {
            for (int direction_sign : {-1, 1}) {
                for (double x_offset = X_start_m; x_offset <= X_end_m; x_offset += step_x_m) {
                    
                    // Tag 坐标系 -> 相机坐标系
                    cv::Mat P_tag = (cv::Mat_<double>(3, 1) << direction_sign * x_offset, y_offset, 0.0);
                    cv::Mat P_cam = R_mat * P_tag + t_mat;

                    double Z_cam = P_cam.at<double>(2, 0);
                    if (Z_cam <= 0.1) continue; 

                    // 投影到像素
                    double u_proj = cx_ + fx_ * (P_cam.at<double>(0, 0) / Z_cam);
                    double v_proj = cy_ + fy_ * (P_cam.at<double>(1, 0) / Z_cam);
                    
                    int u = static_cast<int>(u_proj);
                    int v = static_cast<int>(v_proj);

                    // 简易地面排除 (假设 Tag 不会太歪，用中心点估算底部)
                    // 如果需要更精确，可以将 Tag 4角在 Pose 节点作为 Topic 发出来，或者在这里重新计算 4 角
                    if (v > v_center + (fx_ * half_marker / tag_depth) + 10) continue; 

                    double depth_val = getDepthAtPixel(u, v);
                    if (depth_val < 0.0) continue; 

                    if (depth_val < tag_depth - obstacle_in_front_margin_m_) return true;
                    if (std::abs(depth_val - tag_depth) < depth_similarity_tolerance_m_) return true;
                }
            }
        }
        return false;
    }

    double getDepthAtPixel(int u, int v) {
        if (u < 0 || u >= latest_depth_.cols || v < 0 || v >= latest_depth_.rows) return -1.0;
        if (depth_is_32f_) return static_cast<double>(latest_depth_.at<float>(v, u));
        uint16_t val = latest_depth_.at<uint16_t>(v, u);
        return (val == 0) ? -1.0 : static_cast<double>(val) / 1000.0;
    }

    void publishCollisionMarker(const std_msgs::msg::Header &header, const geometry_msgs::msg::Pose &pose) {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = "aruco_collision";
        m.id = 999;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose = pose;
        m.scale.x = marker_length_ * 1.5; m.scale.y = marker_length_ * 1.5; m.scale.z = marker_length_ * 1.5;
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.5;
        m.lifetime.sec = 0; m.lifetime.nanosec = 200000000;
        collision_marker_pub_->publish(m);
    }
    
    void clearCollisionMarker(const std_msgs::msg::Header &header) {
        visualization_msgs::msg::Marker m;
        m.header = header; m.ns = "aruco_collision"; m.id = 999;
        m.action = visualization_msgs::msg::Marker::DELETE;
        collision_marker_pub_->publish(m);
    }

    // Members
    bool camera_info_received_ = false;
    double fx_, fy_, cx_, cy_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_marker_pub_;

    cv::Mat latest_depth_;
    std::mutex depth_mtx_;
    bool depth_is_32f_ = false;

    // Params
    double marker_length_, grip_allowance_width_m_, depth_similarity_tolerance_m_, obstacle_in_front_margin_m_, x_axis_cargo_buffer_m_, y_axis_check_half_width_m_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionMonitorNode>());
    rclcpp::shutdown();
    return 0;
}