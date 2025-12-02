#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "fairino_msgs/msg/robot_nonrt_state.hpp"

class FairinoTf : public rclcpp::Node
{
public:
    FairinoTf() : Node("fairino_tf_node")
    {
        fairino_nonrt_state_data_sub_ = this->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, std::bind(&FairinoTf::publish_tf_callback, this, std::placeholders::_1));
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        RCLCPP_INFO(this->get_logger(), "维护 机械臂整体 joint_state 节点已启动");
    }

private:
    void publish_tf_callback(const fairino_msgs::msg::RobotNonrtState::SharedPtr msg)
    {
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->now();
        joint_state.name = {"j1", "j2", "j3", "j4", "j5", "j6"};

        // 设置关节位置 (角度转弧度)
        joint_state.position = {
            msg->j1_cur_pos* (M_PI / 180.0),
            msg->j2_cur_pos* (M_PI / 180.0),
            msg->j3_cur_pos* (M_PI / 180.0),
            msg->j4_cur_pos* (M_PI / 180.0),
            msg->j5_cur_pos* (M_PI / 180.0),
            msg->j6_cur_pos* (M_PI / 180.0)
        };

        joint_state_pub_->publish(joint_state);
    }


    rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr fairino_nonrt_state_data_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FairinoTf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}