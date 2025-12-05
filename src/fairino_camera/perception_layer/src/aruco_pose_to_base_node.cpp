#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArucoPoseToBaseNode : public rclcpp::Node
{
public:
    ArucoPoseToBaseNode() : Node("aruco_pose_to_base_node")
    {
        // 参数：机械臂 base_link 坐标系名称
        this->declare_parameter<std::string>("base_link", "base_link");
        this->get_parameter("base_link", base_frame_);

        // TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS image_qos = rclcpp::SensorDataQoS();

        // 订阅相机坐标系下的 pose
        aruco_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "aruco/pose", image_qos,
            std::bind(&ArucoPoseToBaseNode::poseCallback, this, std::placeholders::_1)
        );

        // 发布机械臂 base 坐标系下的 pose
        aruco_pose_base_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "aruco/pose_base", image_qos
        );

        RCLCPP_INFO(this->get_logger(), "ArucoPoseToBaseNode started. Output: /aruco/pose_base");
    }

private:

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose_out;

        try {
            // 将 pose 从相机坐标系 → base 坐标系
            tf_buffer_->transform(*msg, pose_out, base_frame_, tf2::Duration(std::chrono::milliseconds(50)));

            pose_out.header.frame_id = base_frame_;
            pose_out.header.stamp = this->get_clock()->now();
            aruco_pose_base_pub_->publish(pose_out);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Aruco pose in base frame: (%.3f %.3f %.3f)",
                pose_out.pose.position.x,
                pose_out.pose.position.y,
                pose_out.pose.position.z);

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF transform failed: %s", ex.what());
        }
    }

    // 参数
    std::string base_frame_;

    // ROS2 entities
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_base_pub_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoPoseToBaseNode>());
    rclcpp::shutdown();
    return 0;
}
