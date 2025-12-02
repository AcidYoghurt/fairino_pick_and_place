#pragma once
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <future>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.h>
using namespace std::chrono_literals;

class Tool : public rclcpp::Node
{

public:
    Tool() : Node("ToolNode")
    {
        try
        {
            this->declare_parameter("item_above_car_point",std::vector<double>());
            item_above_car_point = this->get_parameter("item_above_car_point").as_double_array();
        } catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),"获取参数失败，请检查custom_points_config文件里面是否填写double类型的数据");
        }

        tcp_msg_pub_ = this->create_publisher<std_msgs::msg::String>("ros_to_tcp_cmd",10);
        fairino_control_client = this->create_client<fairino_msgs::srv::RemoteCmdInterface>("fairino_remote_command_service");
        fairino_nonrt_state_data_sub_ = this->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, std::bind(&Tool::get_task_status, this, std::placeholders::_1));
        item_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("aruco/pose_base", 10,std::bind(&Tool::item_pose_callback, this, std::placeholders::_1));
        item_id_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>("aruco/detected_markers",10,std::bind(&Tool::item_id_callback,this,std::placeholders::_1));

        task_status = 1;
        gripper_status = 1;
        item_id = -1;

        RCLCPP_INFO(this->get_logger(),"工具函数初始化完成！");
    }

    // 发送TCP消息（工具函数）
    void sendTcpMsg(int code,std::string action,std::string message)
    {
        std::string time_str;
        {   // 获取时间
            auto now = std::chrono::system_clock::now();    // 获取当前时间点
            std::time_t t = std::chrono::system_clock::to_time_t(now);  // 转换为 time_t
            std::tm tm = *std::localtime(&t);   // 转换为 tm 结构（本地时间）

            // 格式化输出
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
            time_str = oss.str();
        }

        std_msgs::msg::String str;
        str.data = (nlohmann::json{
            {"code",code},
            {"action", action},
            {"message", message},
            {"datetime", time_str},
            {"data",{}}
        }).dump();
        tcp_msg_pub_->publish(str);
    }

    // 清除错误状态
    void reset_all_error()
    {
        std::string temp_a = "1";
        while ( temp_a != "0"&& rclcpp::ok()&& task_status==1 && gripper_status==1)
        {
            auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
            request->cmd_str = "ResetAllError()";
            auto future = fairino_control_client->async_send_request(request);
            std::future_status status = future.wait_for(3s);
            if (status == std::future_status::ready) {
                auto response = future.get();
                temp_a = response->cmd_res;
                RCLCPP_INFO_THROTTLE(rclcpp::get_logger("Tool"), *this->get_clock(), 1000, "【ResetAllError】服务调用成功：%s", temp_a.data());
                if (temp_a == "0")
                    break;
            } else {
                RCLCPP_ERROR(this->get_logger(), "【ResetAllError】服务调用超时或失败");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // 笛卡尔积坐标走点
    void MoveJ(int task_id,double x, double y,double z,double rx,double ry,double rz)
    {
        std::string temp_a = "1";
        while ( temp_a != "0"&& rclcpp::ok()&& task_status==1&& gripper_status==1)
        {
            auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
            request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+","+std::to_string(rx)+","+std::to_string(ry)+","+std::to_string(rz)+")";
            auto future = fairino_control_client->async_send_request(request);
            std::future_status status = future.wait_for(3s);
            if (status == std::future_status::ready) {
                auto response = future.get();
                temp_a=response->cmd_res;
                RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"【MoveJ】服务调用成功：%s",temp_a.data());
                if (temp_a=="0")
                    break;
            } else {
                RCLCPP_ERROR(this->get_logger(), "【MoveJ】服务调用失败");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::string temp_b = "1";
        while (temp_b != "0"&& rclcpp::ok()&& task_status==1&& gripper_status==1)
        {
            auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
            request_->cmd_str = "MoveJ(CART"+std::to_string(task_id)+",30,1,0,0,0,0,0)";
            auto future_ = fairino_control_client->async_send_request(request_);
            std::future_status status_ = future_.wait_for(3s);
            if (status_ == std::future_status::ready) {
                auto response_ = future_.get();
                temp_b=response_->cmd_res;
                RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"【MoveJ】服务调用成功：%s",temp_b.data());
                if (temp_b=="0")
                    break;
            } else {
                RCLCPP_ERROR(this->get_logger(), "【MoveJ】服务调用失败");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // 笛卡尔积直线
    void MoveL(int task_id,double x, double y,double z,double rx,double ry,double rz)
    {
        std::string temp_a = "1";
        while ( temp_a != "0"&& rclcpp::ok()&& task_status==1&& gripper_status==1)
        {
            auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
            request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+","+std::to_string(rx)+","+std::to_string(ry)+","+std::to_string(rz)+")";
            request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+","+std::to_string(rx)+","+std::to_string(ry)+","+std::to_string(rz)+")";

            auto future = fairino_control_client->async_send_request(request);
            std::future_status status = future.wait_for(3s);
            if (status == std::future_status::ready) {
                auto response = future.get();
                temp_a=response->cmd_res;
                RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"【MoveL】服务调用成功：%s",temp_a.data());
                if (temp_a=="0")
                    break;
            } else {
                RCLCPP_ERROR(this->get_logger(), "【MoveL】服务调用失败");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::string temp_b = "1";
        while (temp_b != "0"&& rclcpp::ok()&& task_status==1&& gripper_status==1)
        {
            auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
            request_->cmd_str = "【MoveL】MoveL(CART"+std::to_string(task_id)+",30,1,0,0,0,0,0)";
            auto future_ = fairino_control_client->async_send_request(request_);
            std::future_status status_ = future_.wait_for(3s);
            if (status_ == std::future_status::ready) {
                auto response_ = future_.get();
                temp_b=response_->cmd_res;
                RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"【MoveL】服务调用成功：%s",temp_b.data());
                if (temp_b=="0")
                    break;
            } else {
                RCLCPP_ERROR(this->get_logger(), "服务调用失败");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // 夹爪控制
    void ControlGripper(int gripper_id,int open_degree)
    {
        while (rclcpp::ok())
        {
            if (task_status==1&& gripper_status==1)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "MoveGripper("+std::to_string(gripper_id)+","+std::to_string(open_degree)+")";
                auto future = fairino_control_client->async_send_request(request);
                std::future_status status = future.wait_for(3s);
                if (status == std::future_status::ready) {
                    auto response = future.get();
                    auto temp_a=response->cmd_res;
                    RCLCPP_INFO(this->get_logger(),"【夹爪】服务调用成功：%s",temp_a.data());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【夹爪】服务调用失败");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // 等待物品
    std::vector<double> WaitItem(long item_id_net,std::string action)
    {
        while (rclcpp::ok() && task_status==1&& gripper_status==1)
        {
            geometry_msgs::msg::PoseStamped current_pose;
            int current_id;
            {
                std::lock_guard<std::mutex> lock(item_pose_mutex_);
                std::lock_guard<std::mutex> lock_(item_id_mutex_);
                current_pose = this->item_pose;
                current_id = this->item_id;
            }
            if (current_pose.header.stamp.sec == 0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "尚未收到物品姿态消息");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            // 物品id校验
            if (item_id_net!=current_id)
            {
                RCLCPP_ERROR(this->get_logger(),"物品ID不匹配！取消执行任务");
                sendTcpMsg(400,action,"物品ID不匹配！取消执行任务");
                MoveJ(1,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],-180,0,-90);    // 去到小车位置【复位】
                return {};
            }

            // 时间戳校验
            if (std::abs((this->get_clock()->now() - current_pose.header.stamp).seconds()) < 0.5)
            {
                // 转换为四元数
                tf2::Quaternion q_original(current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                                          current_pose.pose.orientation.z, current_pose.pose.orientation.w);

                // 创建绕Y轴旋转180度的四元数
                tf2::Quaternion q_rotate_y;
                q_rotate_y.setRPY(0, M_PI, 0);

                // 组合旋转：先原始方向，再绕Y轴旋转180度
                tf2::Quaternion q_result = q_original * q_rotate_y;

                std::vector<double> item_pose = std::vector<double>(6);
                tf2::Matrix3x3 m(q_result);
                m.getRPY(item_pose[3], item_pose[4], item_pose[5]);
                item_pose[3]  = item_pose[3]*180.0/M_PI;
                item_pose[4] = item_pose[4]*180.0/M_PI;
                item_pose[5] = item_pose[5]*180.0/M_PI;
                item_pose[0] = current_pose.pose.position.x*1000;
                item_pose[1] = current_pose.pose.position.y*1000;
                item_pose[2] = current_pose.pose.position.z*1000;
                return item_pose;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000,"正在等待物品");
            }
        }
    }

private:
    void item_pose_callback(geometry_msgs::msg::PoseStamped item_pose)
    {
        std::lock_guard<std::mutex> lock(item_pose_mutex_);
        this->item_pose.header = item_pose.header;
        this->item_pose.pose = item_pose.pose;
    }

    void item_id_callback(std_msgs::msg::Int32MultiArray msg)
    {
        if (!msg.data.empty())
            if (item_id!=msg.data[0])
            {
                std::lock_guard<std::mutex> lock(item_id_mutex_);
                item_id = msg.data[0];
            }
    }

    void get_task_status(fairino_msgs::msg::RobotNonrtState msg)
    {
        task_status = msg.robot_motion_done;
        gripper_status = msg.grip_motion_done;
    }

    std::vector<double> item_above_car_point;

    int task_status;
    int gripper_status;
    rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr fairino_nonrt_state_data_sub_;

    std::mutex item_id_mutex_;
    std::mutex item_pose_mutex_;
    int item_id;
    geometry_msgs::msg::PoseStamped item_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr item_pose_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr item_id_sub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tcp_msg_pub_;
    std::shared_ptr<rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>> fairino_control_client;
};

