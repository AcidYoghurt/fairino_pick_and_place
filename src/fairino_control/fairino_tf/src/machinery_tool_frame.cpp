#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MachineryToolFrame : public rclcpp::Node
{
public:
    MachineryToolFrame() : Node("machinery_tool_frame_node")
    {
        this->declare_parameter("tool_frame_name", "tool_frame");
        this->get_parameter("tool_frame_name", tool_frame_name);
        this->declare_parameter("tool_frame_parent", "wrist3_link");
        this->get_parameter("tool_frame_parent", tool_frame_parent);
        this->declare_parameter("tool_frame_length", 0.16);
        this->get_parameter("tool_frame_length", tool_frame_length);
        this->declare_parameter("roll", 0.0);
        this->get_parameter("roll", roll);
        this->declare_parameter("pitch", 0.0);
        this->get_parameter("pitch", pitch);
        this->declare_parameter("yaw", -0.12217304764);    // -0.12217304764
        this->get_parameter("yaw", yaw);

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->publish_static_transform();
    }

private:
    void publish_static_transform()
    {
        geometry_msgs::msg::TransformStamped t;

        // 父坐标系 和 子坐标系
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = tool_frame_parent;
        t.child_frame_id = tool_frame_name;

        // 平移
        t.transform.translation.z = tool_frame_length;

        // 设置四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // 发布tf
        RCLCPP_INFO(this->get_logger(), "正在发布 %s 到 %s 的静态tf，距离为 %f, RPY为 (%.2f, %.2f, %.2f)",
                    tool_frame_parent.c_str(), tool_frame_name.c_str(), tool_frame_length, roll, pitch, yaw);
        tf_broadcaster_->sendTransform(t);
    }

    double tool_frame_length;
    std::string tool_frame_name;
    std::string tool_frame_parent;
    double roll, pitch, yaw;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MachineryToolFrame>());
    rclcpp::shutdown();
    return 0;
}
