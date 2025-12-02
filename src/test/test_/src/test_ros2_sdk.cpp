#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fairino_msgs/srv/remote_cmd_interface.hpp>
#include <geometry_msgs/msg/detail/accel__traits.hpp>
#include <nlohmann/json.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std::chrono_literals;

class TestRos2Sdk : public rclcpp::Node
{
public:
    TestRos2Sdk() : Node("TestRos2SdkNode")
    {
        task_id=1;
        car_point = std::vector<double>(3);
        point1 = std::vector<double>(3);
        car_point = {500,-100,100};
        point1 = {153,635,128};

        fairino_control_client = this->create_client<fairino_msgs::srv::RemoteCmdInterface>("fairino_remote_command_service");
        item_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("aruco/pose_base",10,std::bind(&TestRos2Sdk::item_pose_callback,this,std::placeholders::_1));
    }

    void item_pose_callback(geometry_msgs::msg::PoseStamped msg)
    {
        item_pose = msg;
    }

    void reset_all_reset()
    {
        auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
        request->cmd_str = "ResetAllError()";
        auto future = fairino_control_client->async_send_request(request);
    }

    void fairino_move()
    {
        {
            {   // 清除错误状态
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "ResetAllError()";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }
            }


            {   // 去到小车位置
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    // // reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(car_point[0])+","+std::to_string(car_point[1])+","+std::to_string(car_point[2])+",-180,0,-90)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveJ(CART"+std::to_string(task_id++)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }

            {   // 打开夹爪
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "MoveGripper(1,0)";
                auto future = fairino_control_client->async_send_request(request);
                auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response = future.get();
                    auto temp_a=response->cmd_res;
                    RCLCPP_INFO(this->get_logger(),"【夹爪】服务调用成功：%s",temp_a.data());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                }
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }

            double item_roll, item_pitch, item_yaw; // 单位为角度
            {   // 等待物品
                while (rclcpp::ok())
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    if (item_pose.header.stamp.sec == 0) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "尚未收到物品姿态消息");
                        continue;
                    }

                    // RCLCPP_INFO(this->get_logger(), "%f", std::abs((this->get_clock()->now() - item_pose.header.stamp).seconds()));
                    if (std::abs((this->get_clock()->now() - item_pose.header.stamp).seconds()) < 0.5)
                    {
                        // 转换为四元数
                        tf2::Quaternion q_original(item_pose.pose.orientation.x, item_pose.pose.orientation.y,
                                                  item_pose.pose.orientation.z, item_pose.pose.orientation.w);

                        // 创建绕Y轴旋转180度的四元数
                        tf2::Quaternion q_rotate_y;
                        q_rotate_y.setRPY(0, M_PI, 0);

                        // 组合旋转：先原始方向，再绕Y轴旋转180度
                        tf2::Quaternion q_result = q_original * q_rotate_y;

                        tf2::Matrix3x3 m(q_result);
                        m.getRPY(item_roll, item_pitch, item_yaw);
                        item_roll  = item_roll*180.0/M_PI;
                        item_pitch = item_pitch*180.0/M_PI;
                        item_yaw = item_yaw*180.0/M_PI;
                        break;
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000,"正在等待物品");
                    }
                }
            }

            {   // 夹爪运动到物品正上方
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    // // reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(item_pose.pose.position.x*1000)+","+std::to_string(item_pose.pose.position.y*1000)+","+std::to_string(car_point[2])+","+std::to_string(item_roll)+","+std::to_string(item_pitch)+","+std::to_string(item_yaw)+")";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // // reset_all_reset();
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveL(CART"+std::to_string(task_id++)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");

                    }
                }
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }

            {   // 夹爪运动到物品（向下移）
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    // // reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(item_pose.pose.position.x*1000)+","+std::to_string(item_pose.pose.position.y*1000)+","+std::to_string(item_pose.pose.position.z*1000)+","+std::to_string(item_roll)+","+std::to_string(item_pitch)+","+std::to_string(item_yaw)+")";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // // reset_all_reset();
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveL(CART"+std::to_string(task_id++)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");

                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            }

            {   // 关闭夹爪（抓取物品）
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "MoveGripper(1,0)";
                auto future = fairino_control_client->async_send_request(request);
                auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response = future.get();
                    auto temp_a=response->cmd_res;
                    RCLCPP_INFO(this->get_logger(),"【夹爪】服务调用成功：%s",temp_a.data());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                }
            }

            {   // 夹爪向上运动
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    // // reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(task_id)+",494,-155,277,-180,0,-90)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // // reset_all_reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveL(CART"+std::to_string(task_id++)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            }

            {   // 去到柜子前面
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    // // reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(point1[0])+","+std::to_string(point1[1])+","+std::to_string(point1[2])+",-90,0,0)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // // reset_all_reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveJ(CART"+std::to_string(task_id++)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            }

            {   // 移动到柜子里面（放置物品）
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    // // reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(point1[0])+","+std::to_string(point1[1]+200)+","+std::to_string(point1[2])+",-90,0,0)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // // reset_all_reset();
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveL(CART"+std::to_string(task_id++)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            }

            {   // 打开夹爪
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "MoveGripper(1,100)";
                auto future = fairino_control_client->async_send_request(request);
                auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response = future.get();
                    auto temp_a=response->cmd_res;
                    RCLCPP_INFO(this->get_logger(),"【夹爪】服务调用成功：%s",temp_a.data());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            }

            {   // 离开柜子
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    // // reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(1)+","+std::to_string(110)+","+std::to_string(600)+","+std::to_string(210)+",-90,0,0)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // // reset_all_reset();
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveL(CART"+std::to_string(1)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            }

            {   // 去到小车位置
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    // // reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(2)+",494,-155,277,-180,0,-90)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");

                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // // reset_all_reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveJ(CART"+std::to_string(2)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");

                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            }
            task_id=1;
        }


        {
            RCLCPP_INFO(this->get_logger(),"物品出柜");
            // TODO：//////////////////////////////////////////
            {   // 去到柜子前面
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    // // reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(1)+","+std::to_string(point1[0])+","+std::to_string(point1[1])+","+std::to_string(point1[2]-20)+",-90,0,0)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // // reset_all_reset();
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveJ(CART"+std::to_string(1)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    }
                }
                std::this_thread::sleep_for(std::chrono::seconds(8));
            }

            {   // 打开夹爪
                 auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                 request->cmd_str = "MoveGripper(1,100)";
                 auto future = fairino_control_client->async_send_request(request);
                 auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                 if (result == rclcpp::FutureReturnCode::SUCCESS) {
                     auto response = future.get();
                     auto temp_a=response->cmd_res;
                     RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                 } else {
                     RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                 }
                 std::this_thread::sleep_for(std::chrono::seconds(2));
            }

            double item_roll, item_pitch, item_yaw; // 单位为角度
            {   // 等待物品
                while (rclcpp::ok())
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    if (item_pose.header.stamp.sec == 0) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "尚未收到物品姿态消息");
                        continue;
                    }

                    // RCLCPP_INFO(this->get_logger(), "%f", std::abs((this->get_clock()->now() - item_pose.header.stamp).seconds()));
                    if (std::abs((this->get_clock()->now() - item_pose.header.stamp).seconds()) < 0.5)
                    {
                        // 转换为四元数
                        tf2::Quaternion q_original(item_pose.pose.orientation.x, item_pose.pose.orientation.y,
                                                  item_pose.pose.orientation.z, item_pose.pose.orientation.w);

                        // 创建绕Y轴旋转180度的四元数
                        tf2::Quaternion q_rotate_y;
                        q_rotate_y.setRPY(0, M_PI, 0);

                        // 组合旋转：先原始方向，再绕Y轴旋转180度
                        tf2::Quaternion q_result = q_original * q_rotate_y;

                        tf2::Matrix3x3 m(q_result);
                        m.getRPY(item_roll, item_pitch, item_yaw);
                        item_roll  = item_roll*180.0/M_PI;
                        item_pitch = item_pitch*180.0/M_PI;
                        item_yaw = item_yaw*180.0/M_PI;
                        break;
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000,"正在等待物品");
                    }
                }
            }

             {   // 夹爪运动到物品前面
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(2)+","+std::to_string(item_pose.pose.position.x*1000)+","+std::to_string(point1[1])+","+std::to_string(item_pose.pose.position.z*1000)+",-90,0,0)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }

                 std::string temp_b = "1";
                 while (temp_b != "0")
                 {
                     reset_all_reset();
                     auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                     request_->cmd_str = "MoveL(CART"+std::to_string(2)+",5,1,0,0,0,0,0)";
                     auto future_ = fairino_control_client->async_send_request(request_);
                     auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                     if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                         auto response_ = future_.get();
                         temp_b=response_->cmd_res;
                         RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                     } else {
                         RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                     }
                 }
                 std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }

            {   // 夹爪运动到物品（向前移动）
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    reset_all_reset();
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(3)+","+std::to_string(item_pose.pose.position.x*1000)+","+std::to_string(item_pose.pose.position.y*1000)+","+std::to_string(item_pose.pose.position.z*1000)+",-90,0,0)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }

                 std::string temp_b = "1";
                 while (temp_b != "0")
                 {
                     // // reset_all_reset();
                     auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                     request_->cmd_str = "MoveL(CART"+std::to_string(3)+",30,1,0,0,0,0,0)";
                     auto future_ = fairino_control_client->async_send_request(request_);
                     auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                     if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                         auto response_ = future_.get();
                         temp_b=response_->cmd_res;
                         RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                     } else {
                         RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                     }
                 }
                 std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }

            {   // 关闭夹爪（抓取物品）
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "MoveGripper(1,0)";
                auto future = fairino_control_client->async_send_request(request);
                auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response = future.get();
                    std::string temp_a=response->cmd_res;
                    RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }

            {   // 离开柜子
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(4)+","+std::to_string(point1[0])+","+std::to_string(point1[1])+","+std::to_string(point1[2])+","+std::to_string(item_roll)+","+std::to_string(item_pitch)+","+std::to_string(item_yaw)+")";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // // reset_all_reset();
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveJ(CART"+std::to_string(4)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            }

            {   // 去到小车上
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(5)+","+std::to_string(car_point[0])+","+std::to_string(car_point[1])+","+std::to_string(car_point[2])+",-180,0,-90)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    // reset_all_reset();
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveL(CART"+std::to_string(5)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            }

            {   // 把物品放到小车正上方（夹爪下降）
                std::string temp_a = "1";
                while (temp_a != "0")
                {
                    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "CARTPoint("+std::to_string(6)+","+std::to_string(car_point[0])+","+std::to_string(car_point[1])+","+std::to_string(car_point[2]-200)+",-180,0,-90)";
                    auto future = fairino_control_client->async_send_request(request);
                    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response = future.get();
                        temp_a=response->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                    }
                }
                std::string temp_b = "1";
                while (temp_b != "0")
                {
                    reset_all_reset();
                    auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request_->cmd_str = "MoveL(CART"+std::to_string(6)+",30,1,0,0,0,0,0)";
                    auto future_ = fairino_control_client->async_send_request(request_);
                    auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                    if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                        auto response_ = future_.get();
                        temp_b=response_->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "服务调用失败");

                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            }

            {   // 打开夹爪
                 auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                 request->cmd_str = "MoveGripper(1,100)";
                 auto future = fairino_control_client->async_send_request(request);
                 auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                 if (result == rclcpp::FutureReturnCode::SUCCESS) {
                     auto response = future.get();
                     auto temp_a=response->cmd_res;
                     RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                 } else {
                     RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                 }
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            {   // 回到小车上
                 std::string temp_a = "1";
                 while (temp_a != "0")
                 {
                     auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                     request->cmd_str = "CARTPoint("+std::to_string(7)+","+std::to_string(car_point[0])+","+std::to_string(car_point[1])+","+std::to_string(car_point[2])+",-180,0,-90)";
                     auto future = fairino_control_client->async_send_request(request);
                     auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,1s);
                     if (result == rclcpp::FutureReturnCode::SUCCESS) {
                         auto response = future.get();
                         temp_a=response->cmd_res;
                         RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_a.data());
                     }
                 }
                 std::string temp_b = "1";
                 while (temp_b != "0")
                 {
                     // reset_all_reset();
                     auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                     request_->cmd_str = "MoveL(CART"+std::to_string(7)+",30,1,0,0,0,0,0)";
                     auto future_ = fairino_control_client->async_send_request(request_);
                     auto result_ = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_,1s);
                     if (result_ == rclcpp::FutureReturnCode::SUCCESS) {
                         auto response_ = future_.get();
                         temp_b=response_->cmd_res;
                         RCLCPP_INFO(this->get_logger(),"服务调用成功：%s",temp_b.data());
                     }
                 }
            }
            task_id=1;
        }
    }

private:
    std::shared_ptr<rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>> fairino_control_client;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr item_pose_sub;

    int task_id;
    std::vector<double> car_point;
    std::vector<double> point1;
    geometry_msgs::msg::PoseStamped item_pose;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestRos2Sdk>();
    node->fairino_move();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}