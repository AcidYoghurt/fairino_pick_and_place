#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <future>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <fairino_msgs/srv/remote_cmd_interface.hpp>
#include <fairino_msgs/msg/robot_nonrt_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
using namespace std::chrono_literals;


class FairinoRos2sdkControl : public rclcpp::Node
{
public:
    FairinoRos2sdkControl():Node("FairinoRos2sdkControlNode")
    {
        task_status.store(1);
        gripper_status.store(1);
        is_running=false;

        try
        {
            this->declare_parameter("item_above_car_point",std::vector<double>());
            item_above_car_point = this->get_parameter("item_above_car_point").as_double_array();

            this->declare_parameter("item_in_car_point",std::vector<double>());
            item_in_car_point = this->get_parameter("item_in_car_point").as_double_array();

            std::string yaml_str;
            this->declare_parameter("cabinet_front_points","");
            this->get_parameter("cabinet_front_points", yaml_str);
            YAML::Node node = YAML::Load(yaml_str);
            for (auto pt : node) {
                cabinet_front_points.push_back(pt.as<std::vector<double>>());
            }

            this->declare_parameter("cabinet_inside_points","");
            this->get_parameter("cabinet_inside_points", yaml_str);
            node = YAML::Load(yaml_str);
            for (auto pt : node) {
                cabinet_inside_points.push_back(pt.as<std::vector<double>>());
            }

            this->declare_parameter("cabinet_recognize_points","");
            this->get_parameter("cabinet_recognize_points", yaml_str);
            node = YAML::Load(yaml_str);
            for (auto pt : node) {
                cabinet_recognize_points.push_back(pt.as<std::vector<double>>());
            }

            // 点在车上的夹爪角度
            // 左A：180,0,-90
            // 右B：-180,0,-90
            this->declare_parameter("car_point_posture", std::vector<double>({180,0,-90}));
            this->get_parameter("car_point_posture",car_point_posture);

            // 点在柜子的夹爪角度
            // 左A：90,0,180
            // 右B：-90,0,0
            this->declare_parameter("cabinet_point_posture", std::vector<double>({90,0,180}));
            this->get_parameter("cabinet_point_posture",cabinet_point_posture);

            // 夹爪抓取物品时x轴的旋转期望（相对于世界坐标系）
            this->declare_parameter("desired_gripper_rotation_x",std::vector<double>({0,-1,0}));  // 默认值表示此时夹爪的x是世界坐标系的y的负方向
            this->get_parameter("desired_gripper_rotation_x",desired_gripper_rotation_x);

        } catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),"获取参数失败，请检查custom_points_config文件里面是否填写double类型的数据");
        }

        // QOS策略
        rclcpp::QoS keepLast_qos(rclcpp::KeepLast(100));
        keepLast_qos.reliable();
        rclcpp::QoS image_qos = rclcpp::SensorDataQoS();
        rclcpp::QoS state_qos = rclcpp::SensorDataQoS();

        client_node_ = rclcpp::Node::make_shared("fairino_client_node");
        tcp_msg_pub_ = this->create_publisher<std_msgs::msg::String>("ros_to_tcp_cmd",keepLast_qos);
        tcp_msg_sub_ = this->create_subscription<std_msgs::msg::String>("tcp_to_ros_cmd",keepLast_qos,std::bind(&FairinoRos2sdkControl::tcp_callback,this,std::placeholders::_1));
        fairino_nonrt_state_data_sub_ = this->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", state_qos, std::bind(&FairinoRos2sdkControl::get_task_status, this, std::placeholders::_1));
        fairino_control_client = client_node_->create_client<fairino_msgs::srv::RemoteCmdInterface>("fairino_remote_command_service");
        item_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("aruco/pose_base", image_qos,std::bind(&FairinoRos2sdkControl::item_pose_callback, this, std::placeholders::_1));
        item_id_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>("aruco/detected_markers",image_qos,std::bind(&FairinoRos2sdkControl::item_id_callback,this,std::placeholders::_1));
        this->basic_setting();  // 初始化
        RCLCPP_INFO(this->get_logger(),"机械臂控制节点已启动！");
    }

private:
    // 执行任务序列
    void tcp_callback(std_msgs::msg::String msg)
    {
        /* 接收网络信息 */
        std::string task_type="";
        std::string action="";
        long item_id_net=-1;
        std::pair<int,int> cabinet_id = std::pair<int,int>(-1,-1);
        // 解析json数据
        try
        {
            nlohmann::json j = nlohmann::json::parse(msg.data);
            int code = j.value("code",400);
            if (code==200)
            {
                action = j.value("action","");
                if (action=="place")
                {
                    // 解析 data 里面的数据
                    task_type = j["data"]["task_type"];
                    if (task_type=="item_store")
                    {
                        // 非必要
                        item_id_net = j["data"]["item_id"];
                        //必要
                        cabinet_id.first = j["data"]["cabinet_id"][0];
                        cabinet_id.second = j["data"]["cabinet_id"][1];
                    }
                    else if (task_type=="item_outbound")
                    {
                        //非必要
                        item_id_net = j["data"]["item_id"];
                        cabinet_id.first = j["data"]["cabinet_id"][0];
                        cabinet_id.second = j["data"]["cabinet_id"][1];
                    }
                }
                else if (action=="pick_up")
                {
                    // 解析 data 里面的数据
                    task_type = j["data"]["task_type"];
                    if (task_type=="item_store")
                    {
                        // 非必要
                        cabinet_id.first = j["data"]["cabinet_id"][0];
                        cabinet_id.second = j["data"]["cabinet_id"][1];
                        //必要
                        item_id_net = j["data"]["item_id"];
                    }
                    else if (task_type=="item_outbound")
                    {
                        //必要
                        item_id_net = j["data"]["item_id"];
                        cabinet_id.first = j["data"]["cabinet_id"][0];
                        cabinet_id.second = j["data"]["cabinet_id"][1];
                    }
                }
                else if (action=="open_gripper" || action=="close_gripper"){}
                else
                    return;
            }
            else
            {
                sendTcpMsg(400,action,"code值非法,不接受任务",task_type,item_id_net,cabinet_id);
                return;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"json字段中的data错误: %s",e.what());
            sendTcpMsg(400,action,"json字段中的data错误: "+std::string(e.what()),task_type,item_id_net,cabinet_id);
            return;
        }

        /* 执行任务 */
        if (!is_running)
        {
            std::thread(&FairinoRos2sdkControl::fairino_move_thread, this,task_type,action,item_id_net,cabinet_id).detach();
        }else
        {
            RCLCPP_WARN(this->get_logger(),"正在执行任务，忽略此任务！");
            sendTcpMsg(400,action,"正在执行任务，忽略此任务！",task_type,item_id_net,cabinet_id);
        }
    }
    void fairino_move_thread(std::string task_type,std::string action,long item_id_net,std::pair<int,int> cabinet_id)
    {
        struct StateGuard {
            std::atomic<bool>& flag;
            StateGuard(std::atomic<bool>& f) : flag(f) { flag = true; }
            ~StateGuard() { flag = false; }
        } state_guard(is_running);

        /* 任务序列 */
        if (task_type=="item_store")
        {
            RCLCPP_INFO(this->get_logger(),"物品入柜");
            if (action=="pick_up")
            {
                int task_id=1;
                RCLCPP_INFO(this->get_logger(),"正在执行抓取物品");
                if (!reset_all_error(task_type,action,item_id_net,cabinet_id))
                    return;
                if (!MoveJ(task_id++,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2]))     // 去到小车
                    return;
                if (!ControlGripper(action,task_type,item_id_net,cabinet_id,1,100))   // 打开夹爪
                    return;
                std::vector<double> item_pose = WaitItem(action,task_type,item_id_net,cabinet_id);
                if (item_pose.empty())
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],(item_above_car_point[2]+item_pose[2])/3,car_point_posture[0],car_point_posture[1],car_point_posture[2]))     // 机械臂下降（运动到物品前方）
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,item_pose[0],item_pose[1],(item_above_car_point[2]+item_pose[2])/3,car_point_posture[0],car_point_posture[1],item_pose[5]))   // 机械臂向前（运动到物品上方）
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,item_pose[0],item_pose[1],item_pose[2],car_point_posture[0],car_point_posture[1],item_pose[5]))  // 夹爪运动到物品（机械臂向下移）
                    return;
                if (!ControlGripper(action,task_type,item_id_net,cabinet_id,1,0))     // 关闭夹爪
                    return;
                if (!MoveJ(task_id++,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2]))     // 回到小车
                    return;
                TaskSuccess(action,task_type,item_id_net,cabinet_id);
            }
            else if (action=="place")
            {
                int task_id=1;
                RCLCPP_INFO(this->get_logger(),"正在执行放置物品");
                if (!reset_all_error(action,task_type,item_id_net,cabinet_id))   // 清除错误状态
                    return;
                if (!MoveJ(task_id++,action,task_type,item_id_net,cabinet_id,cabinet_front_points[cabinet_id.second][0],cabinet_front_points[cabinet_id.second][1],cabinet_front_points[cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2]))   // 去到柜子前
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,cabinet_inside_points[cabinet_id.second][0],cabinet_inside_points[cabinet_id.second][1],cabinet_inside_points[cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2]))    // 移动到柜子里面（放置物品）
                    return;
                if (!ControlGripper(action,task_type,item_id_net,cabinet_id,1,100))  // 打开夹爪
                    return;
                if (!MoveJ(task_id++,action,task_type,item_id_net,cabinet_id,cabinet_front_points[cabinet_id.second][0],cabinet_front_points[cabinet_id.second][1],cabinet_front_points[cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2]))  // 出去柜子
                    return;
                if (!MoveJ(task_id++,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2]))     // 去到小车上
                    return;
                TaskSuccess(action,task_type,item_id_net,cabinet_id);
            }
        }
        else if (task_type=="item_outbound")
        {
            RCLCPP_INFO(this->get_logger(),"物品出柜");
            if (action=="pick_up")
            {
                int task_id=1;
                RCLCPP_INFO(this->get_logger(),"正在执行抓取物品");
                if (!reset_all_error(action,task_type,item_id_net,cabinet_id))
                    return;
                if (!ControlGripper(action,task_type,item_id_net,cabinet_id,1,100))   // 打开夹爪
                    return;
                // if (!MoveJ(task_id++,action, cabinet_front_points[cabinet_id.second][0],cabinet_front_points[cabinet_id.second][1],cabinet_front_points[cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2]))      //去到柜子前面
                //     return;
                if (!MoveJ(task_id++,action,task_type,item_id_net,cabinet_id,cabinet_recognize_points[cabinet_id.second][0],cabinet_recognize_points[cabinet_id.second][1],cabinet_recognize_points[cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2]))   // 去到物品识别点位
                    return;
                std::vector<double> item_pose = WaitItem(action,task_type,item_id_net,cabinet_id);    //等待物品
                if (item_pose.empty())
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,item_pose[0],cabinet_recognize_points[cabinet_id.second][1],item_pose[2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2]))   // 移动到物品前面
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,item_pose[0],item_pose[1],item_pose[2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2])) // 移动到物品
                    return;
                if (!ControlGripper(action,task_type,item_id_net,cabinet_id,1,0)) // 关闭夹爪
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,cabinet_front_points[cabinet_id.second][0],cabinet_front_points[cabinet_id.second][1],cabinet_front_points[cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2]))   // 离开柜子
                    return;
                if (!MoveJ(task_id++,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2])) // 去到小车上
                    return;
                TaskSuccess(action,task_type,item_id_net,cabinet_id);
            }
            if (action=="place")
            {
                int task_id=1;
                RCLCPP_INFO(this->get_logger(),"正在执行放置物品");
                if (!reset_all_error(action,task_type,item_id_net,cabinet_id))
                    return;
                if (!MoveJ(task_id++,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2])) // 去到小车上
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],(item_above_car_point[2]+item_in_car_point[2])/3,car_point_posture[0],car_point_posture[1],car_point_posture[2]))     // 机械臂下降
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,item_in_car_point[0],item_in_car_point[1],(item_above_car_point[2]+item_in_car_point[2])/3,car_point_posture[0],car_point_posture[1],car_point_posture[2]))   // 机械臂向前（运动到物品前方）
                    return;
                if (!MoveL(task_id++,action,task_type,item_id_net,cabinet_id,item_in_car_point[0],item_in_car_point[1],item_in_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2]))  // 把物品放到小车正上方（夹爪下降）
                    return;
                if (!ControlGripper(action,task_type,item_id_net,cabinet_id,1,100))   // 打开夹爪
                    return;
                if (!MoveJ(task_id++,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2])) // 去到小车上
                    return;
                TaskSuccess(action,task_type,item_id_net,cabinet_id);
            }
        }
        else if (action=="open_gripper")
        {
            RCLCPP_INFO(this->get_logger(),"正在演示demo：打开夹爪");
            if (!reset_all_error(action,task_type,item_id_net,cabinet_id))
                return;
            if (!ControlGripper(action,task_type,item_id_net,cabinet_id,1,100))   // 打开夹爪
                return;
            TaskSuccess(action,task_type,item_id_net,cabinet_id);
        }
        else if (action=="close_gripper")
        {
            RCLCPP_INFO(this->get_logger(),"正在演示demo：闭合夹爪");
            if (!reset_all_error(action,task_type,item_id_net,cabinet_id))
                return;
            if (!ControlGripper(action,task_type,item_id_net,cabinet_id,1,0))   // 关闭夹爪
                return;
            TaskSuccess(action,task_type,item_id_net,cabinet_id);
        }
    }

    // 获取任务状态
    void get_task_status(fairino_msgs::msg::RobotNonrtState msg)
    {
        task_status = msg.robot_motion_done;
        // gripper_status = msg.grip_motion_done;
        // RCLCPP_INFO(this->get_logger(),"任务状态：%d,%d",static_cast<int>(task_status),static_cast<int>(gripper_status));
        j6_cur_pose = msg.j6_cur_pos;
    }

    // 获取要抓取的物品 id 和 位姿
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

    // 发送TCP消息（工具函数）
    void sendTcpMsg(int code,std::string action,std::string message,std::string task_type,long item_id,std::pair<int,int> cabinet_id)
    {
        std::string time_str;
        {   // 获取时间
            auto now = std::chrono::system_clock::now();    // 获取当前时间点
            std::time_t t = std::chrono::system_clock::to_time_t(now);  // 转换为 time_t
            std::tm tm = *std::localtime(&t);   // 转换为 tm 结构（本地时间）

            // 格式化输出
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d_%H_%M_%S");
            time_str = oss.str();
        }

        std_msgs::msg::String str;
        str.data = (nlohmann::json{
            {"code",code},
            {"action", action},
            {"message", message},
            {"datetime", time_str},
            {"data",{{"task_type",task_type},{"item_id",item_id},{"cabinet_id",{cabinet_id.first,cabinet_id.second}}}}
        }).dump();
        tcp_msg_pub_->publish(str);
    }

    /* 任务序列函数 */
    // 基础设置
    void basic_setting()
    {
        while (!fairino_control_client->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "正在等待法奥机械臂服务端启动中...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        // 使能
        std::string temp_a = "1";
        while ( temp_a != "0"&& rclcpp::ok())
        {
            auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
            request->cmd_str = "RobotEnable(1)";
            auto future = fairino_control_client->async_send_request(request);
            auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
            if (result == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = future.get();
                temp_a = response->cmd_res;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "【BasicSetting】服务调用成功：%s", temp_a.data());
                if (temp_a=="0")
                    break;
            } else {
                RCLCPP_ERROR(this->get_logger(), "【BasicSetting】服务调用超时或失败");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // 设置手动模式
        while ( temp_a != "0"&& rclcpp::ok())
        {
            auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
            request->cmd_str = "Mode(1)";
            auto future = fairino_control_client->async_send_request(request);
            auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
            if (result == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = future.get();
                temp_a = response->cmd_res;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "【BasicSetting】服务调用成功：%s", temp_a.data());
                if (temp_a=="0")
                    break;
            } else {
                RCLCPP_ERROR(this->get_logger(), "【BasicSetting】服务调用超时或失败");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // 清除错误状态
    bool reset_all_error(std::string action,std::string task_type,long item_id_net,std::pair<int,int> cabinet_id)
    {
        // 使能
        int try_count = 1;
        std::string temp_a = "1";
        while ( temp_a != "0"&& rclcpp::ok())
        {
            if (task_status==1 && gripper_status==1)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "RobotEnable(1)";
                auto future = fairino_control_client->async_send_request(request);
                auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response = future.get();
                    temp_a = response->cmd_res;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "【ResetAllError】服务调用成功：%s", temp_a.data());
                    if (temp_a=="0")
                        break;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【ResetAllError】服务调用超时或失败");
                }
                try_count++;
            }
            if (try_count>=5)
            {
                RCLCPP_ERROR(this->get_logger(),"超过最大尝试次数，任务失败！");
                sendTcpMsg(400,action,"超过最大尝试次数，任务失败！",task_type,item_id_net,cabinet_id);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // 清除错误状态
        try_count = 1;
        temp_a = "1";
        while ( temp_a != "0"&& rclcpp::ok())
        {
            if (task_status==1 && gripper_status==1)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "ResetAllError()";
                auto future = fairino_control_client->async_send_request(request);
                auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response = future.get();
                    temp_a = response->cmd_res;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "【ResetAllError】服务调用成功：%s", temp_a.data());
                    if (temp_a == "0")
                        break;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【ResetAllError】服务调用超时或失败");
                }
                try_count++;
            }
            if (try_count>=5)
            {
                RCLCPP_ERROR(this->get_logger(),"超过最大尝试次数，任务失败！");
                sendTcpMsg(400,action,"超过最大尝试次数，任务失败！",task_type,item_id_net,cabinet_id);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        return true;
    }

    // 笛卡尔积坐标走点
    bool MoveJ(int task_id,std::string action,std::string task_type,long item_id_net,std::pair<int,int> cabinet_id,double x, double y,double z,double rx,double ry,double rz,int speed=30)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        int try_count = 1;
        std::string temp_a = "1";
        while ( temp_a != "0"&& rclcpp::ok())
        {
            if (task_status.load()==1 && gripper_status.load()==1)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+","+std::to_string(rx)+","+std::to_string(ry)+","+std::to_string(rz)+")";
                auto future = fairino_control_client->async_send_request(request);
                auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response = future.get();
                    temp_a=response->cmd_res;
                    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"【MoveJ】服务调用成功：%s",temp_a.data());
                    if (temp_a=="0")
                        break;
                    reset_all_error(action,task_type,item_id_net,cabinet_id);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【MoveJ】服务调用失败");
                }
                try_count++;
            }
            if (try_count>=5)
            {
                RCLCPP_ERROR(this->get_logger(),"超过最大尝试次数，任务失败！");
                sendTcpMsg(400,action,"超过最大尝试次数，任务失败！",task_type,item_id_net,cabinet_id);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        try_count = 1;
        std::string temp_b = "1";
        while (temp_b != "0"&& rclcpp::ok())
        {
            if (task_status.load()==1 && gripper_status.load()==1)
            {
                auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request_->cmd_str = "MoveJ(CART"+std::to_string(task_id)+","+std::to_string(speed)+",1,0,0,0,0,0)";
                auto future_ = fairino_control_client->async_send_request(request_);
                auto result = rclcpp::spin_until_future_complete(client_node_, future_, std::chrono::seconds(5));
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response_ = future_.get();
                    temp_b=response_->cmd_res;
                    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"【MoveJ】服务调用成功：%s",temp_b.data());
                    if (temp_b=="0")
                        break;
                    reset_all_error(action,task_type,item_id_net,cabinet_id);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【MoveJ】服务调用失败");
                }
                try_count++;
            }
            if (try_count>=5)
            {
                RCLCPP_ERROR(this->get_logger(),"超过最大尝试次数，任务失败！");
                sendTcpMsg(400,action,"超过最大尝试次数，任务失败！",task_type,item_id_net,cabinet_id);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        return true;
    }

    // 笛卡尔积直线
    bool MoveL(int task_id,std::string action,std::string task_type,long item_id_net,std::pair<int,int> cabinet_id,double x, double y,double z,double rx,double ry,double rz,int speed=30)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        int try_count = 1;
        std::string temp_a = "1";
        while ( temp_a != "0"&& rclcpp::ok())
        {
            if (task_status.load()==1 && gripper_status.load()==1)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+","+std::to_string(rx)+","+std::to_string(ry)+","+std::to_string(rz)+")";

                auto future = fairino_control_client->async_send_request(request);
                auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response = future.get();
                    temp_a=response->cmd_res;
                    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"【MoveL】服务调用成功：%s",temp_a.data());
                    if (temp_a=="0")
                        break;
                    reset_all_error(action,task_type,item_id_net,cabinet_id);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【MoveL】服务调用失败");
                }
                try_count++;
            }
            if (try_count>=5)
            {
                RCLCPP_ERROR(this->get_logger(),"超过最大尝试次数，任务失败！");
                sendTcpMsg(400,action,"超过最大尝试次数，任务失败！",task_type,item_id_net,cabinet_id);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        try_count=1;
        std::string temp_b = "1";
        while (temp_b != "0"&& rclcpp::ok())
        {
            if (task_status.load()==1 && gripper_status.load()==1)
            {
                auto request_ = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request_->cmd_str = "MoveL(CART"+std::to_string(task_id)+","+std::to_string(speed)+",1,0,0,0,0,0)";
                auto future_ = fairino_control_client->async_send_request(request_);
                auto result = rclcpp::spin_until_future_complete(client_node_, future_, std::chrono::seconds(5));
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response_ = future_.get();
                    temp_b=response_->cmd_res;
                    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"【MoveL】服务调用成功：%s",temp_b.data());
                    if (temp_b=="0")
                        break;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "服务调用失败");
                }
                try_count++;
            }
            if (try_count>=5)
            {
                RCLCPP_ERROR(this->get_logger(),"超过最大尝试次数，任务失败！");
                sendTcpMsg(400,action,"超过最大尝试次数，任务失败！",task_type,item_id_net,cabinet_id);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        return true;
    }

    // 夹爪控制
    bool ControlGripper(std::string action,std::string task_type,long item_id_net,std::pair<int,int> cabinet_id,int gripper_id,int open_degree)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        while (rclcpp::ok())
        {
            if (task_status.load()==1 && gripper_status.load()==1)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "MoveGripper("+std::to_string(gripper_id)+","+std::to_string(open_degree)+")";
                auto future = fairino_control_client->async_send_request(request);
                auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto response = future.get();
                    auto temp_a=response->cmd_res;
                    if (temp_a=="0")
                    {
                        RCLCPP_INFO(this->get_logger(),"【夹爪】服务调用成功：%s",temp_a.data());
                        return true;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(),"【夹爪】服务调用失败：%s，请重新插拔夹爪线",temp_a.data());
                        sendTcpMsg(400,action,"夹爪服务调用失败："+temp_a+"，请重新插拔夹爪线",task_type,item_id_net,cabinet_id);
                        return false;
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【夹爪】服务调用失败，请重新插拔夹爪线");
                    sendTcpMsg(400,action,"夹爪服务调用失败，请重新插拔夹爪线",task_type,item_id_net,cabinet_id);
                    return false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // 等待物品
    std::vector<double> WaitItem(std::string action,std::string task_type,long item_id_net,std::pair<int,int> cabinet_id)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_WARN(this->get_logger(),"等待物品");
        rclcpp::Time past_time = this->get_clock()->now();
        while (rclcpp::ok())
        {
            if (task_status.load()==1&& gripper_status.load()==1)
            {
                // 获取物品id和pose
                geometry_msgs::msg::PoseStamped current_pose;
                long current_id;
                {
                    std::scoped_lock lock(item_pose_mutex_, item_id_mutex_);
                    current_pose = this->item_pose;
                    current_id = this->item_id;
                }

                // 时间戳校验
                if (std::abs((this->get_clock()->now() - current_pose.header.stamp).seconds()) > 0.5) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "尚未收到物品姿态消息");
                    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                // 物品id校验
                else if (item_id_net!=current_id)
                {
                    RCLCPP_ERROR(this->get_logger(),"物品ID不匹配，待抓取ID为%ld，实际物品ID为%ld！取消执行任务",item_id_net,current_id);
                    sendTcpMsg(400,action,"物品ID不匹配，待抓取ID为"+std::to_string(item_id_net)+"，实际物品ID为"+std::to_string(current_id)+"！取消执行任务",task_type,item_id_net,cabinet_id);
                    MoveJ(1,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2]);    // 去到小车位置【复位】
                    return {};
                }
                // 两次校验都成功
                else
                {
                    // 获取物品原始姿态
                    tf2::Quaternion q_original(
                        current_pose.pose.orientation.x,
                        current_pose.pose.orientation.y,
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);

                    // 物品绕x轴旋转180°，使得物品Z轴朝下
                    tf2::Quaternion q_flip;
                    q_flip.setRPY(M_PI, 0, 0);    // 绕X翻180度：物体向上 → 向下
                    tf2::Quaternion q_after_flip = q_original * q_flip;
                    q_after_flip.normalize();

                    // 比较物品的x轴和夹爪的x轴（向量点积）
                    tf2::Matrix3x3 m1(q_after_flip);
                    tf2::Vector3 obj_x = m1.getColumn(0);   // 物体X轴（翻面后）
                    tf2::Vector3 grip_x(desired_gripper_rotation_x[0],desired_gripper_rotation_x[1],desired_gripper_rotation_x[2]);            // 夹爪在世界坐标系中的期望方向向量
                    double dot = grip_x.dot(obj_x);         // 物品x轴与夹爪x轴同向时为正，否则为负

                    // 如果物品与夹爪期望相反（向量点积）
                    if(dot < 0)
                    {
                        tf2::Quaternion q_align;
                        q_align.setRPY(0, 0, M_PI);        // 再绕Z旋转180°
                        q_after_flip = q_after_flip * q_align;
                        q_after_flip.normalize();

                        RCLCPP_INFO(this->get_logger(),"翻面后方向相反，绕Z自动旋转180°");
                    }

                    // 最终计算 RPY
                    std::vector<double> item_pose(6);
                    tf2::Matrix3x3 m(q_after_flip);
                    m.getRPY(item_pose[3],item_pose[4],item_pose[5]);

                    item_pose[3] = item_pose[3] * 180/M_PI;
                    item_pose[4] = item_pose[4] * 180/M_PI;
                    item_pose[5] = item_pose[5] * 180/M_PI;

                    item_pose[0] = current_pose.pose.position.x * 1000;
                    item_pose[1] = current_pose.pose.position.y * 1000;
                    item_pose[2] = current_pose.pose.position.z * 1000;

                    // 避免 yaw 进入 ±180°，不可达姿态
                    if (std::abs(item_pose[5]) > 178)
                    {
                        RCLCPP_WARN(this->get_logger(), "Yaw 接近 ±180°，已自动修正为 0°");
                        item_pose[5] = 0;
                    }

                    return item_pose;
                }
            }else
            {
                RCLCPP_WARN(this->get_logger(),"机械臂正在运动");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                past_time = this->get_clock()->now();
            }

            // 等待物品超时
            if ((this->get_clock()->now()-past_time).seconds()>=30)
            {
                RCLCPP_ERROR(this->get_logger(),"等待物品超时，超时时间为30s");
                sendTcpMsg(400,action,"等待物品超时",task_type,item_id_net,cabinet_id);
                MoveJ(1,action,task_type,item_id_net,cabinet_id,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2]);    // 去到小车位置【复位】
                return {};
            }
        }
    }

    // 任务完成
    void TaskSuccess(std::string action,std::string task_type,long item_id_net,std::pair<int,int> cabinet_id)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        while (rclcpp::ok())
        {
            if (task_status.load()==1 && gripper_status.load()==1)
            {
                RCLCPP_INFO(this->get_logger(),"任务执行成功");
                sendTcpMsg(200,action,"任务执行成功",task_type,item_id_net,cabinet_id);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    std::vector<double> desired_gripper_rotation_x;
    std::vector<double> item_above_car_point;
    std::vector<double> item_in_car_point;
    std::vector<std::vector<double>> cabinet_front_points;
    std::vector<std::vector<double>> cabinet_inside_points;
    std::vector<std::vector<double>> cabinet_recognize_points;
    std::vector<double> car_point_posture;
    std::vector<double> cabinet_point_posture;

    std::atomic<int> task_status;
    std::atomic<int> gripper_status;
    std::atomic<double> j6_cur_pose;
    std::atomic<bool> is_running;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tcp_msg_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tcp_msg_pub_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr fairino_nonrt_state_data_sub_;
    std::shared_ptr<rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>> fairino_control_client;

    std::mutex item_id_mutex_;
    std::mutex item_pose_mutex_;
    int item_id;
    geometry_msgs::msg::PoseStamped item_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr item_pose_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr item_id_sub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FairinoRos2sdkControl>();

    // 使用多线程执行器，允许同时处理订阅数据和服务响应
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}