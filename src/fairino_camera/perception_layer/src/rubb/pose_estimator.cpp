#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <fairino_msg/msg/qr_msg.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> // 用于姿态计算
#include <tf2/convert.h> 

class QrPoseEstimatorNode : public rclcpp::Node
{
public:
    QrPoseEstimatorNode() : Node("qr_pose_estimator_node")
    {
        // 1. TF2 初始化
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // 2. 相机信息订阅
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/depth_rect_to_color/camera_info",
            rclcpp::QoS(10).reliable(), 
            std::bind(&QrPoseEstimatorNode::cameraInfoCallback, this, std::placeholders::_1));

        // 3. 消息过滤器订阅者
        qr_msg_sub_ = std::make_shared<message_filters::Subscriber<fairino_msg::msg::QrMsg>>(this, "qr/qr_msg");
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/depth_rect_to_color/image");
        
        // 4. 同步策略
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<fairino_msg::msg::QrMsg, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), 
            *qr_msg_sub_, 
            *depth_sub_
        );
        sync_->registerCallback(std::bind(&QrPoseEstimatorNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        // 5. 发布者
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("qr/item_pose_world", 10);
        RCLCPP_INFO(this->get_logger(), "QrPoseEstimatorNode 初始化完成。");
    }

private:
    // 类型定义
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<fairino_msg::msg::QrMsg, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // 订阅者/发布者/TF
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::shared_ptr<message_filters::Subscriber<fairino_msg::msg::QrMsg>> qr_msg_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // 状态/内参
    bool camera_info_received_ = false;
    double fx, fy, cx, cy;
    std::string camera_frame_id_;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_)
        {
            fx = msg->k[0];
            fy = msg->k[4];
            cx = msg->k[2];
            cy = msg->k[5];
            camera_frame_id_ = msg->header.frame_id;
            RCLCPP_INFO(this->get_logger(), "相机内参已接收 (Frame ID: %s)。", camera_frame_id_.c_str());
            camera_info_received_ = true;
        }
    }
    
    // 辅助函数：将 2D 像素点转换为 3D 相机坐标系下的点
    geometry_msgs::msg::Point pixel_to_3d_point(int u, int v, uint16_t depth_value)
    {
        geometry_msgs::msg::Point p;
        if (depth_value == 0) {
            p.x = p.y = p.z = 0.0;
            return p;
        }

        // 深度相机的深度值单位是mm，转换为m
        double Z_cam = static_cast<double>(depth_value) / 1000.0; 
        p.z = Z_cam;
        p.x = (u - cx) * Z_cam / fx;
        p.y = (v - cy) * Z_cam / fy;
        return p;
    }

    void syncCallback(
    const fairino_msg::msg::QrMsg::ConstSharedPtr qr_msg, 
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
    {
        if (!camera_info_received_ || qr_msg->corners.size() != 4)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "等待相机内参或角点数量不足 (当前: %zu)...", qr_msg->corners.size());
            return;
        }

        // --- 1. 转深度图 ---
        cv_bridge::CvImagePtr depth_ptr;
        try {
            depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转化异常: %s", e.what());
            return;
        }
        cv::Mat depth_image = depth_ptr->image;

        // --- 2. 3D 重建 ---
        std::vector<geometry_msgs::msg::Point> pts(4);
        geometry_msgs::msg::Point center_cam;

        for (int i = 0; i < 4; i++)
        {
            int u = qr_msg->corners[i].x;
            int v = qr_msg->corners[i].y;

            if (u < 0 || u >= depth_image.cols || v < 0 || v >= depth_image.rows) {
                RCLCPP_WARN(this->get_logger(), "像素越界");
                return;
            }

            uint16_t d = depth_image.at<uint16_t>(v, u);
            pts[i] = pixel_to_3d_point(u, v, d);

            if (pts[i].z == 0) {
                RCLCPP_WARN(this->get_logger(), "深度为0");
                return;
            }

            center_cam.x += pts[i].x;
            center_cam.y += pts[i].y;
            center_cam.z += pts[i].z;
        }
        center_cam.x /= 4.0;
        center_cam.y /= 4.0;
        center_cam.z /= 4.0;

        // ==========================================================
        // === 3. 正确姿态计算：使用两个边向量 + 法向量构建坐标系 ===
        // ==========================================================

        // 假设角点顺序：P0=左上, P1=右上, P2=右下, P3=左下
        tf2::Vector3 p0(pts[0].x, pts[0].y, pts[0].z);
        tf2::Vector3 p1(pts[1].x, pts[1].y, pts[1].z);
        tf2::Vector3 p3(pts[3].x, pts[3].y, pts[3].z);

        // 局部 X (P0->P1) 和 Y (P0->P3)
        tf2::Vector3 x_axis = p1 - p0;
        tf2::Vector3 y_axis = p3 - p0;

        x_axis.normalize();
        y_axis.normalize();

        // 局部 Z：法向量
        tf2::Vector3 z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        // 正交化：重新计算 y_axis
        y_axis = z_axis.cross(x_axis);
        y_axis.normalize();

        // 构建相机坐标系下的旋转矩阵
        tf2::Matrix3x3 R_cam(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z()
        );

        // 转四元数
        tf2::Quaternion q_cam;
        R_cam.getRotation(q_cam);
        q_cam.normalize();

        // --- 4. TF 转换到 world ---
        geometry_msgs::msg::PointStamped pt_cam;
        pt_cam.header.stamp = depth_msg->header.stamp;
        pt_cam.header.frame_id = camera_frame_id_;
        pt_cam.point = center_cam;

        geometry_msgs::msg::PointStamped pt_world;

        std::string target_frame = "world";
        geometry_msgs::msg::TransformStamped T_cam_to_world;

        try {
            T_cam_to_world = tf_buffer->lookupTransform(
                target_frame, pt_cam.header.frame_id,
                depth_msg->header.stamp,
                rclcpp::Duration::from_seconds(0.1));

            tf2::doTransform(pt_cam, pt_world, T_cam_to_world);

            tf2::Quaternion q_tf, q_world;
            tf2::fromMsg(T_cam_to_world.transform.rotation, q_tf);

            // 世界方向 = 世界←相机 * 相机局部姿态
            q_world = q_tf * q_cam;
            q_world.normalize();

            // 若你确实只需要水平旋转（忽略 roll/pitch）
            double roll, pitch, yaw;
            tf2::Matrix3x3(q_world).getRPY(roll, pitch, yaw);

            tf2::Quaternion q_yaw_only;
            q_yaw_only.setRPY(0, 0, yaw);

            // --- 5. 发布 Pose ---
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = pt_world.header.stamp;
            pose.header.frame_id = target_frame;
            pose.pose.position = pt_world.point;
            pose.pose.orientation = tf2::toMsg(q_yaw_only);

            pose_publisher_->publish(pose);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Yaw = %.2f deg | Pos = (%.3f %.3f %.3f)",
                yaw * 180.0 / M_PI,
                pt_world.point.x, pt_world.point.y, pt_world.point.z
            );

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
        }
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QrPoseEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}