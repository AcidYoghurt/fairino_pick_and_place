#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticTFBroadcaster : public rclcpp::Node
{
public:
    StaticTFBroadcaster() : Node("static_tf_broadcaster")
    {
        // 初始化静态TF广播器
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // 发布静态变换
        publish_static_tf();
    }

private:
    void publish_static_tf()
    {
        // 创建TransformStamped消息
        geometry_msgs::msg::TransformStamped transform;

        // 设置时间戳和坐标系ID
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "world";  // 父坐标系
        transform.child_frame_id = "base_link"; // 子坐标系

        // 设置平移量 (单位：米)
        transform.transform.translation.x = 0.5;
        transform.transform.translation.y = 0.3;
        transform.transform.translation.z = 0.6;

        // 设置旋转量 (通过欧拉角转换为四元数)
        tf2::Quaternion q;
        q.setRPY(0,M_PI,0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // 发布变换
        broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "已发布静态TF: '%s' 相对于 '%s'",
                    transform.child_frame_id.c_str(),
                    transform.header.frame_id.c_str());
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}