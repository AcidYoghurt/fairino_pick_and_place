// aruco_detector_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <map>
#include <deque>
#include <mutex>
#include <cmath>
#include <string>
#include <algorithm> // for std::max, std::min

class ArucoDetectorNode : public rclcpp::Node {
public:
    ArucoDetectorNode()
        : Node("aruco_detector_node")
    {
        // --- 参数声明 (通用参数) ---
        // ArUco 标记的物理边长（米）。
        declare_parameter<double>("marker_length", 0.05);
        // 位姿平滑系数 (α)。
        declare_parameter<double>("smoothing_alpha", 0.2);
        // 稳定性检测的窗口大小（帧数）。
        declare_parameter<int>("stability_window_size", 5);
        // 稳定性位置阈值（米）。
        declare_parameter<double>("stability_position_threshold", 0.04);
        // 彩色图像 ROS 话题名称。
        declare_parameter<std::string>("color_image_topic", "camera/color/image_raw");
        // 彩色相机内参信息 ROS 话题名称。
        declare_parameter<std::string>("color_camera_info_topic", "camera/color/camera_info");
        // 深度图像 ROS 话题名称。
        declare_parameter<std::string>("depth_image_topic", "/camera/depth/image_raw");

        // --- 碰撞检测参数 ---
        // 夹爪沿 X 轴方向需要的总净空距离（米）。
        declare_parameter<double>("x_axis_grip_allowance_m", 0.02); 
        // 深度相似容差（米）。用于侧面碰撞检测。
        declare_parameter<double>("depth_similarity_tolerance_m", 0.02); 
        // 前方障碍物判断裕度（米）。
        declare_parameter<double>("obstacle_in_front_margin_m", 0.02); 
        // 货物缓冲距离（米）。从 Tag 边缘向外延伸的距离。
        declare_parameter<double>("x_axis_cargo_buffer_m", 0.01); 
        // 沿 Tag Y 轴方向的检测半宽度（米）。
        declare_parameter<double>("y_axis_check_half_width_m", 0.01); 

        get_parameter("marker_length", marker_length_);
        get_parameter("smoothing_alpha", alpha_);
        get_parameter("stability_window_size", stability_window_size_);
        get_parameter("stability_position_threshold", stability_pos_thres_);
        get_parameter("color_image_topic", color_image_topic_);
        get_parameter("color_camera_info_topic", color_camera_info_topic_);
        get_parameter("depth_image_topic", depth_image_topic_);
        
        // 获取碰撞检测参数
        get_parameter("x_axis_grip_allowance_m", grip_allowance_width_m_);
        get_parameter("depth_similarity_tolerance_m", depth_similarity_tolerance_m_);
        get_parameter("obstacle_in_front_margin_m", obstacle_in_front_margin_m_);
        get_parameter("x_axis_cargo_buffer_m", x_axis_cargo_buffer_m_);
        get_parameter("y_axis_check_half_width_m", y_axis_check_half_width_m_);

        // --- 订阅与发布 ---
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            color_camera_info_topic_, 10,
            std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            color_image_topic_, 10,
            std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            depth_image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetectorNode::depthCallback, this, std::placeholders::_1));

        annotated_pub_ = create_publisher<sensor_msgs::msg::Image>("aruco/image", 10);
        pose_pub_      = create_publisher<geometry_msgs::msg::PoseStamped>("aruco/pose", 10);
        id_pub_        = create_publisher<std_msgs::msg::Int32MultiArray>("aruco/detected_markers", 10);
        marker_pub_    = create_publisher<visualization_msgs::msg::Marker>("aruco/marker", 10);
        collision_pub_ = create_publisher<visualization_msgs::msg::Marker>("aruco/collision_marker", 10);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
        detector_params_ = cv::aruco::DetectorParameters::create();

        RCLCPP_INFO(get_logger(), "Node Started. Collision Check Params: Buffer=%.3fm, Allowance=%.3fm, Y-Half-Width=%.3fm, Tol=%.3fm",
                    x_axis_cargo_buffer_m_, grip_allowance_width_m_, y_axis_check_half_width_m_, depth_similarity_tolerance_m_);
    }

private:
    // ---------------- Camera Info ----------------
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_) {
            fx_ = msg->k[0]; fy_ = msg->k[4];
            cx_ = msg->k[2]; cy_ = msg->k[5];

            camera_matrix_ = (cv::Mat1d(3,3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
            dist_coeffs_ = cv::Mat(msg->d).clone();
            camera_info_received_ = true;
            RCLCPP_INFO(get_logger(), "Camera Info Received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx_, fy_, cx_, cy_);
        }
    }

    // ---------------- 深度回调（保存最近一帧深度） ----------------
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(depth_mtx_);
        try {
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                msg->encoding == "16UC1") {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                latest_depth_ = cv_ptr->image.clone(); // uint16 (mm or mm-like)
                depth_is_32f_ = false;
            } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                       msg->encoding == "32FC1") {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
                latest_depth_ = cv_ptr->image.clone(); // float meters
                depth_is_32f_ = true;
            } else {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
                if (cv_ptr->image.type() == CV_16UC1) {
                    latest_depth_ = cv_ptr->image.clone();
                    depth_is_32f_ = false;
                } else {
                    cv::Mat tmp;
                    cv_ptr->image.convertTo(tmp, CV_32F);
                    latest_depth_ = tmp.clone();
                    depth_is_32f_ = true;
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "Depth conversion failed: %s", e.what());
        }
    }

    // ---------------- 深度获取辅助函数 ----------------
    double getDepthAtPixel(int u, int v)
    {
        if (u < 0 || u >= latest_depth_.cols || v < 0 || v >= latest_depth_.rows) {
            return -1.0; // Invalid pixel
        }

        double depth_val = 0.0;
        
        // 获取深度值
        if (!depth_is_32f_) {
            uint16_t val = latest_depth_.at<uint16_t>(v, u);
            if (val == 0) return -1.0; // Invalid depth
            depth_val = static_cast<double>(val) / 1000.0;
        } else {
            float val = latest_depth_.at<float>(v, u);
            if (std::isnan(val) || val <= 0.0f) return -1.0; // Invalid depth
            depth_val = static_cast<double>(val);
        }
        return depth_val;
    }

    // ---------------- 可视化绘制函数 ----------------
    void drawCollisionCheckArea(cv::Mat& image)
    {
        // 绘制所有采样的像素点 (Tag X 轴两侧的检测区域)
        for (const auto& p : sampled_pixels_) {
            // 使用小矩形/方块来表示采样点，颜色设置为橙蓝色 (cv::Scalar(255, 165, 0))
            cv::rectangle(image, 
                          cv::Point(p.x - 1, p.y - 1), 
                          cv::Point(p.x + 1, p.y + 1), 
                          cv::Scalar(255, 165, 0), 
                          -1); // 填充
        }
    }


    // ----------------  图像处理主逻辑 ----------------
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!camera_info_received_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (...) { return; }

        cv::Mat image = cv_ptr->image;
        cv::Mat annotated = image.clone();

        // 1. 检测所有 ArUco
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        cv::aruco::detectMarkers(image, dictionary_, corners, ids, detector_params_);

        if (ids.empty()) {
            resetStability(); 
            annotated_pub_->publish(*cv_ptr->toImageMsg());
            publishEmptyMarkers(msg->header);
            return;
        }

        // 2. 找到离图像中心最近的一个 Marker
        int best_idx = -1;
        double min_dist_sq = std::numeric_limits<double>::max();
        cv::Point2f center_pixel(cx_, cy_);

        for (size_t i = 0; i < ids.size(); ++i) {
            cv::Point2f marker_center(0, 0);
            for (const auto& p : corners[i]) marker_center += p;
            marker_center *= 0.25f;

            double dist_sq = cv::norm(marker_center - center_pixel);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
            }
        }

        if (best_idx == -1) return;

        // 3. 只对最近的一个 Marker 进行处理
        int target_id = ids[best_idx];
        std::vector<std::vector<cv::Point2f>> target_corner = {corners[best_idx]};

        std_msgs::msg::Int32MultiArray id_msg;
        id_msg.data.push_back(target_id);
        id_pub_->publish(id_msg);

        cv::aruco::drawDetectedMarkers(annotated, target_corner, std::vector<int>{target_id});

        // Pose 解算
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(target_corner, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        // 4. 构建原始 Pose
        geometry_msgs::msg::PoseStamped raw_pose;
        raw_pose.header = msg->header;
        raw_pose.pose.position.x = tvecs[0][0];
        raw_pose.pose.position.y = tvecs[0][1];
        raw_pose.pose.position.z = tvecs[0][2];

        // rvec -> quaternion
        cv::Mat R; cv::Rodrigues(rvecs[0], R);
        tf2::Matrix3x3 tf_R(
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
        );
        tf2::Quaternion q;
        tf_R.getRotation(q);
        raw_pose.pose.orientation = tf2::toMsg(q);

        cv::aruco::drawAxis(annotated, camera_matrix_, dist_coeffs_, rvecs[0], tvecs[0], marker_length_ * 0.5);

        // 5. 碰撞检测
        bool collision = false;
        if (checkCollisionRisk(target_corner[0], rvecs[0], tvecs[0])) {
            collision = true;
            cv::putText(annotated, "COLLISION_RISK", corners[best_idx][0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);
        }
        
        // 5.5. 绘制碰撞检测区域 (可视化)
        drawCollisionCheckArea(annotated);

        // 6. 稳定性检测与滤波
        geometry_msgs::msg::PoseStamped out_pose; 
        bool published_pose = false;
        if (!collision) {
            if (checkStabilityAndFilter(target_id, raw_pose, out_pose)) {
                pose_pub_->publish(out_pose);
                published_pose = true;
                cv::putText(annotated, "STABLE", corners[best_idx][0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);
            } else {
                cv::putText(annotated, "UNSTABLE", corners[best_idx][0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 2);
            }
        } 

        // 发布图像和 Marker
        cv_bridge::CvImage out;
        out.header = msg->header;
        out.encoding = "bgr8";
        out.image = annotated;
        annotated_pub_->publish(*out.toImageMsg());

        if (published_pose) {
            publishPoseMarker(out.header, target_id, out_pose.pose, collision);
        } else {
            publishPoseMarker(raw_pose.header, target_id, raw_pose.pose, collision);
        }

        if (collision) {
            publishCollisionMarker(raw_pose.header, target_id, raw_pose.pose);
        } else {
            clearCollisionMarker(raw_pose.header, target_id);
        }
    }


    // ----------------  主碰撞检测函数 (包含 3D 迭代和可视化数据收集) ----------------

    bool checkCollisionRisk(const std::vector<cv::Point2f>& corners, const cv::Vec3d& rvec, const cv::Vec3d& tvec)
    {
        std::lock_guard<std::mutex> lock(depth_mtx_);
        if (latest_depth_.empty()) {
            RCLCPP_WARN_ONCE(get_logger(), "Depth image not received yet. Collision check skipped.");
            return false;
        }

        double tag_depth = tvec[2]; // Tag 距离相机的 Z 轴距离

        // 1. 获取 Tag 投影的最小/最大边界 (Bounding Box)，用于地面排除
        cv::Point2f min_pt = corners[0];
        cv::Point2f max_pt = corners[0];
        for (const auto& p : corners) {
            min_pt.x = std::min(min_pt.x, p.x);
            min_pt.y = std::min(min_pt.y, p.y);
            max_pt.x = std::max(max_pt.x, p.x);
            max_pt.y = std::max(max_pt.y, p.y);
        }
        float lowest_tag_y = max_pt.y; 
        
        
        // --- PART 1: 路径障碍物区域扫描 (前方大范围路径检测) ---
        int pad_px = 20; 
        int min_x = std::max(0, (int)min_pt.x - pad_px);
        int max_x = std::min(latest_depth_.cols - 1, (int)max_pt.x + pad_px);
        int min_y = std::max(0, (int)min_pt.y - pad_px);
        int max_y = std::min(latest_depth_.rows - 1, (int)max_pt.y + pad_px);

        // 新增：限制最大纵坐标，排除地面或桌面 (上次的修改)
        // 注意：如果相机倾斜大，10像素可能不够，您可以尝试更大的值，如 30
        float front_scan_ground_exclusion_margin = 10.0f; // 保持 10 像素
        int max_y_no_ground = std::min(max_y, (int)lowest_tag_y + (int)front_scan_ground_exclusion_margin);

        for (int v = min_y; v <= max_y_no_ground; ++v) { 
            for (int u = min_x; u <= max_x; ++u) {
                
               
                // 只有当采样点 u, v 在 Tag 投影框外时，才进行深度检查。
                if (u >= (int)min_pt.x && u <= (int)max_pt.x && 
                    v >= (int)min_pt.y && v <= (int)max_pt.y) 
                {
                    // 采样点位于 Tag 标记的投影区域内，跳过检查
                    continue; 
                }
                
                double depth_val = getDepthAtPixel(u, v);
                if (depth_val > 0.0 && depth_val < tag_depth - obstacle_in_front_margin_m_) {
                     RCLCPP_WARN(get_logger(), "Collision Path Front (Area Scan): d=%.3f < tag=%.3f px=(%d,%d)", depth_val, tag_depth, u, v);
                     last_obs_depth_ = depth_val; last_obs_px_ = cv::Point(u,v);
                     return true; 
                }
            }
        }
        
        // --- PART 2: 夹爪 X 轴方向侧面检测 (使用 3D 迭代) ---
        
        // 清除上一帧的采样点
        sampled_pixels_.clear();

        // 准备 Tag 坐标系到相机坐标系的变换
        cv::Mat R_mat;
        cv::Rodrigues(rvec, R_mat); // R_mat is 3x3
        cv::Mat t_mat = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]); // T is 3x1

        // 定义检测的物理距离范围 (米)
        double half_marker = marker_length_ * 0.5;
        double X_start_m = half_marker + x_axis_cargo_buffer_m_;
        double X_end_m = half_marker + grip_allowance_width_m_;
        
        // 采样步长
        double step_x_m = 0.01; // X 轴采样间隔 (1 厘米)
        double step_y_m = 0.02; // Y 轴采样间隔 (2 厘米)
        
        // Y 轴检测半范围
        double Y_half_range = y_axis_check_half_width_m_;

        // 1. 遍历 Y 轴范围 (Tag 坐标系的垂直方向)
        for (double y_offset = -Y_half_range; y_offset <= Y_half_range; y_offset += step_y_m) {
            
            // 2. 遍历 X 轴正方向 (direction_sign = 1) 和负方向 (direction_sign = -1)
            for (int direction_sign : {-1, 1}) {
                // 3. 遍历 X 轴偏移量 (从缓冲起始点到夹爪终点)
                for (double x_offset = X_start_m; x_offset <= X_end_m; x_offset += step_x_m) {
                    
                    // Tag 坐标系下的采样点: (X 轴偏移, Y 轴偏移, Z=0)
                    cv::Mat P_tag = (cv::Mat_<double>(3, 1) << direction_sign * x_offset, y_offset, 0.0);
                    
                    // 转换到 相机坐标系: P_cam = R * P_tag + T
                    cv::Mat P_cam = R_mat * P_tag + t_mat;

                    // 投影到 2D 图像平面
                    double Z_cam = P_cam.at<double>(2, 0);
                    if (Z_cam <= 0.0 || Z_cam < 0.1) continue; 
                    
                    double u_proj = cx_ + fx_ * (P_cam.at<double>(0, 0) / Z_cam);
                    double v_proj = cy_ + fy_ * (P_cam.at<double>(1, 0) / Z_cam);

                    int u = static_cast<int>(u_proj);
                    int v = static_cast<int>(v_proj);

                    // 存储像素点用于可视化
                    sampled_pixels_.emplace_back(u, v);
                    
                    // 地面排除
                    float ground_exclusion_margin = 10.0f; 
                    if (v > lowest_tag_y + ground_exclusion_margin) continue;

                    // 获取深度
                    double depth_val = getDepthAtPixel(u, v);
                    if (depth_val < 0.0) continue; 
                    
                    // 碰撞检查
                    
                    // 6a. 障碍物在 Tag 前方 (明显比 Tag 更近)
                    if (depth_val < tag_depth - obstacle_in_front_margin_m_) { 
                         RCLCPP_WARN(get_logger(), "Collision Front (3D Sample): d=%.3f < tag=%.3f (Offset=%.3f/%.3f) Dir=%d", 
                                     depth_val, tag_depth, x_offset, y_offset, direction_sign);
                         last_obs_depth_ = depth_val; last_obs_px_ = cv::Point(u,v);
                         return true;
                    }
                    
                    // 6b. 障碍物在 Tag 旁边 (深度相似，夹爪可能撞到)
                    else if (std::abs(depth_val - tag_depth) < depth_similarity_tolerance_m_) {
                         RCLCPP_WARN(get_logger(), "Collision Side (3D Sample): d=%.3f ~= tag=%.3f (Offset=%.3f/%.3f) Dir=%d", 
                                     depth_val, tag_depth, x_offset, y_offset, direction_sign);
                         last_obs_depth_ = depth_val; last_obs_px_ = cv::Point(u,v);
                         return true;
                    }
                }
            }
        }

        return false;
    }


    // ----------------  RViz marker 发布/稳定性检查 (略) ----------------
    void publishPoseMarker(const std_msgs::msg::Header &header, int id, const geometry_msgs::msg::Pose &pose, bool collision)
    {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = "aruco_tag";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose = pose;
        m.scale.x = marker_length_;
        m.scale.y = marker_length_;
        m.scale.z = 0.01; 
        if (collision) {
            m.color.r = 0.8; m.color.g = 0.4; m.color.b = 0.0; m.color.a = 1.0;
        } else {
            m.color.r = 0.0; m.color.g = 0.8; m.color.b = 0.0; m.color.a = 0.9;
        }
        m.lifetime = makeDuration(0.5);

        marker_pub_->publish(m);
    }

    void publishCollisionMarker(const std_msgs::msg::Header &header, int id, const geometry_msgs::msg::Pose &pose)
    {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = "aruco_collision";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose = pose;
        m.scale.x = marker_length_ * 1.2;
        m.scale.y = marker_length_ * 1.2;
        m.scale.z = marker_length_ * 1.5;
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
        m.lifetime = makeDuration(0.5);

        collision_pub_->publish(m);

        if (last_obs_depth_ > 0.0 && last_obs_px_.x >= 0) {
            visualization_msgs::msg::Marker s;
            s.header = header;
            s.ns = "aruco_obs_point";
            s.id = id + 10000;
            s.type = visualization_msgs::msg::Marker::SPHERE;
            s.action = visualization_msgs::msg::Marker::ADD;
            geometry_msgs::msg::Point p;
            double z = last_obs_depth_;
            double x = (last_obs_px_.x - cx_) * z / fx_;
            double y = (last_obs_px_.y - cy_) * z / fy_;
            p.x = x; p.y = y; p.z = z;
            s.pose.position = p;
            s.scale.x = 0.02; s.scale.y = 0.02; s.scale.z = 0.02;
            s.color.r = 1.0; s.color.g = 0.2; s.color.b = 0.2; s.color.a = 1.0;
            s.lifetime = makeDuration(0.5);

            collision_pub_->publish(s);
        }
    }

    void clearCollisionMarker(const std_msgs::msg::Header &header, int id)
    {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = "aruco_collision";
        m.id = id;
        m.action = visualization_msgs::msg::Marker::DELETE;
        collision_pub_->publish(m);

        visualization_msgs::msg::Marker s;
        s.header = header;
        s.ns = "aruco_obs_point";
        s.id = id + 10000;
        s.action = visualization_msgs::msg::Marker::DELETE;
        collision_pub_->publish(s);
    }

    void publishEmptyMarkers(const std_msgs::msg::Header &header)
    {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = "aruco_collision";
        m.id = 0;
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        collision_pub_->publish(m);
        m.ns = "aruco_tag";
        marker_pub_->publish(m);
    }

    bool checkStabilityAndFilter(int id, const geometry_msgs::msg::PoseStamped &in, geometry_msgs::msg::PoseStamped &out_pose)
    {
        std::lock_guard<std::mutex> lock(filter_mtx_);

        if (current_tracking_id_ != id) {
            pose_buffer_.clear();
            current_tracking_id_ = id;
        }

        pose_buffer_.push_back(in);
        if (pose_buffer_.size() > (size_t)stability_window_size_) {
            pose_buffer_.pop_front();
        }

        if (pose_buffer_.size() < (size_t)stability_window_size_) {
            return false;
        }

        const auto &newest = pose_buffer_.back().pose.position;
        for (const auto &p : pose_buffer_) {
            double dx = p.pose.position.x - newest.x;
            double dy = p.pose.position.y - newest.y;
            double dz = p.pose.position.z - newest.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist > stability_pos_thres_) {
                return false;
            }
        }

        if (!has_last_stable_pose_) {
            last_stable_pose_ = in;
            has_last_stable_pose_ = true;
            out_pose = in;
            return true;
        }

        geometry_msgs::msg::PoseStamped smoothed = in;
        smoothed.pose.position.x = alpha_ * in.pose.position.x + (1 - alpha_) * last_stable_pose_.pose.position.x;
        smoothed.pose.position.y = alpha_ * in.pose.position.y + (1 - alpha_) * last_stable_pose_.pose.position.y;
        smoothed.pose.position.z = alpha_ * in.pose.position.z + (1 - alpha_) * last_stable_pose_.pose.position.z;

        tf2::Quaternion q_curr, q_prev;
        tf2::fromMsg(in.pose.orientation, q_curr);
        tf2::fromMsg(last_stable_pose_.pose.orientation, q_prev);

        if (q_curr.dot(q_prev) < 0.0) {
            q_curr = tf2::Quaternion(-q_curr.x(), -q_curr.y(), -q_curr.z(), -q_curr.w());
        }

        tf2::Quaternion q_out = q_curr.slerp(q_prev, 1.0 - alpha_);
        smoothed.pose.orientation = tf2::toMsg(q_out);

        last_stable_pose_ = smoothed;
        out_pose = smoothed;
        return true;
    }

    void resetStability() {
        std::lock_guard<std::mutex> lock(filter_mtx_);
        pose_buffer_.clear();
        current_tracking_id_ = -1;
        has_last_stable_pose_ = false;
    }

    // ---------------- 工具函数 ----------------

    builtin_interfaces::msg::Duration makeDuration(double sec)
    {
        builtin_interfaces::msg::Duration d;
        d.sec = (int)sec;
        d.nanosec = (sec - d.sec) * 1e9;
        return d;
    }

    // ---------------- Members ----------------
    // 内参
    bool camera_info_received_ = false;
    double fx_, fy_, cx_, cy_;
    cv::Mat camera_matrix_, dist_coeffs_;

    // 参数
    double marker_length_;
    double alpha_;
    int stability_window_size_;
    double stability_pos_thres_;

    std::string color_image_topic_, color_camera_info_topic_, depth_image_topic_;
    
    // 碰撞检测的新参数
    double grip_allowance_width_m_;
    double depth_similarity_tolerance_m_;
    double obstacle_in_front_margin_m_; 
    double x_axis_cargo_buffer_m_; 
    double y_axis_check_half_width_m_; // 新增 Y 轴检测半宽度

    // 采样点缓存（用于可视化）
    std::vector<cv::Point2i> sampled_pixels_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr id_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_pub_;

    // ArUco
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    // depth buffer
    cv::Mat latest_depth_;
    std::mutex depth_mtx_;
    bool depth_is_32f_ = false;
    double last_obs_depth_ = -1.0;
    cv::Point last_obs_px_ = cv::Point(-1,-1);

    // 滤波与稳定性数据
    std::mutex filter_mtx_;
    int current_tracking_id_ = -1;
    std::deque<geometry_msgs::msg::PoseStamped> pose_buffer_;
    geometry_msgs::msg::PoseStamped last_stable_pose_;
    bool has_last_stable_pose_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}