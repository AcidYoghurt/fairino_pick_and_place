#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <fairino_msgs/srv/remote_cmd_interface.hpp>
#include <fairino_msgs/msg/robot_nonrt_state.hpp>

// 定义 Task 结构体
struct TaskData
{
    std::string task_type;                      // 任务类型：
    std::string action;                         // 动作：place 或 pick_up
    long item_id = -1;                          // 要操作的物品的ID
    std::pair<int,int> cabinet_id{-1,-1};   // 要去往的柜子ID
};

class FairinoRos2sdkControl : public rclcpp::Node
{
public:
    rclcpp::Node::SharedPtr client_node_;   //独立 client node，专门用于法奥SDK调用（相当于多线程）

    FairinoRos2sdkControl():Node("FairinoRos2sdkControlNode")
    {
        last_status.store(1);
        this_status.store(1);
        machineryTCP_current_pose = std::vector<double>(6);
        machine_status = SeqState::IDLE;
        is_running=false;

        try
        {
            // 机械臂速度
            this->declare_parameter("speed",speed);
            speed = this->get_parameter("speed").as_int();

            // 原点
            this->declare_parameter("item_above_car_point",std::vector<double>());
            item_above_car_point = this->get_parameter("item_above_car_point").as_double_array();

            // 机械臂放置物品到车上的点位
            this->declare_parameter("item_in_car_point",std::vector<double>());
            item_in_car_point = this->get_parameter("item_in_car_point").as_double_array();

            // 机械臂到达柜子前面点位【顺序是从上往下增大】
            std::string yaml_str;
            this->declare_parameter("cabinet_front_points","");
            this->get_parameter("cabinet_front_points", yaml_str);
            YAML::Node node = YAML::Load(yaml_str);
            for (auto pt : node) {
                cabinet_front_points.push_back(pt.as<std::vector<double>>());
            }

            // std::vector<std::string> yaml_str;
            // this->declare_parameter("cabinet_front_points", std::vector<std::string>());
            // this->get_parameter("cabinet_front_points", yaml_str);
            // for (const auto &s : yaml_str)
            // {
            //     auto pt = YAML::Load(s).as<std::vector<double>>();
            //     cabinet_front_points.push_back(pt);
            // }

            // 进入柜子放置物品点位
            this->declare_parameter("cabinet_inside_points","");
            this->get_parameter("cabinet_inside_points", yaml_str);
            node = YAML::Load(yaml_str);
            for (auto pt : node) {
                cabinet_inside_points.push_back(pt.as<std::vector<double>>());
            }

            // this->declare_parameter("cabinet_inside_points", std::vector<std::string>());
            // this->get_parameter("cabinet_inside_points", yaml_str);
            // for (const auto &s : yaml_str)
            // {
            //     auto pt = YAML::Load(s).as<std::vector<double>>();
            //     cabinet_inside_points.push_back(pt);
            // }

            // 在柜子前识别物品点位
            this->declare_parameter("cabinet_recognize_points","");
            this->get_parameter("cabinet_recognize_points", yaml_str);
            node = YAML::Load(yaml_str);
            for (auto pt : node) {
                cabinet_recognize_points.push_back(pt.as<std::vector<double>>());
            }

            // this->declare_parameter("cabinet_recognize_points", std::vector<std::string>());
            // this->get_parameter("cabinet_recognize_points", yaml_str);
            // for (const auto &s : yaml_str)
            // {
            //     auto pt = YAML::Load(s).as<std::vector<double>>();
            //     cabinet_recognize_points.push_back(pt);
            // }

            // 点在车上的夹爪角度
            // 左A：180,0,-90
            // 右B：-180,0,-90
            this->declare_parameter("car_point_posture", std::vector<double>({-180,0,-90}));
            this->get_parameter("car_point_posture",car_point_posture);

            // 点在柜子的夹爪角度
            // 左A：90,0,180
            // 右B：-90,0,0
            this->declare_parameter("cabinet_point_posture", std::vector<double>({-90,0,0}));
            this->get_parameter("cabinet_point_posture",cabinet_point_posture);

            // 夹爪抓取物品时x轴的旋转期望（相对于世界坐标系）
            this->declare_parameter("desired_gripper_rotation_x",std::vector<double>({0,-1,0}));  // 默认值表示此时夹爪的x是世界坐标系的y的负方向
            this->get_parameter("desired_gripper_rotation_x",desired_gripper_rotation_x);
        } catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),"获取参数失败，请检查custom_points_config文件里面是否填写double类型的数据，具体错误：%s",e.what());
        }

        // QOS策略
        rclcpp::QoS keepLast_qos(rclcpp::KeepLast(100));
        keepLast_qos.reliable();
        rclcpp::QoS state_qos(rclcpp::KeepLast(10));
        state_qos.reliable();

        client_node_ = rclcpp::Node::make_shared("fairino_client_node");
        tcp_msg_pub_ = this->create_publisher<std_msgs::msg::String>("ros_to_tcp_cmd",keepLast_qos);
        tcp_msg_sub_ = this->create_subscription<std_msgs::msg::String>("tcp_to_ros_cmd",keepLast_qos,std::bind(&FairinoRos2sdkControl::tcp_callback,this,std::placeholders::_1));
        fairino_nonrt_state_data_sub_ = this->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", state_qos, std::bind(&FairinoRos2sdkControl::get_task_status, this, std::placeholders::_1));
        fairino_control_client = client_node_->create_client<fairino_msgs::srv::RemoteCmdInterface>("fairino_remote_command_service");
        item_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("aruco/pose_base", 10,std::bind(&FairinoRos2sdkControl::item_pose_callback, this, std::placeholders::_1));
        item_id_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>("aruco/detected_markers",10,std::bind(&FairinoRos2sdkControl::item_id_callback,this,std::placeholders::_1));
        // 一次性 timer：确保 spin 后再初始化服务调用，避免构造阶段 future 超时
        startup_init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                startup_init_timer_->cancel();
                this->basic_setting();
            });
        RCLCPP_INFO(this->get_logger(),"机械臂控制节点已启动！");
    }

private:
    // 执行任务序列
    void tcp_callback(const std_msgs::msg::String& msg)
    {
        /* 接收网络信息 */
        auto task = std::make_shared<TaskData>();
        // 解析json数据
        try
        {
            nlohmann::json j = nlohmann::json::parse(msg.data);
            int code = j.value("code",400);
            if (code==200)
            {
                task->action = j.value("action","");
                if (task->action=="place")
                {
                    // 解析 data 里面的数据
                    task->task_type = j["data"]["task_type"];
                    if (task->task_type=="item_store")
                    {
                        // 非必要
                        task->item_id = j["data"]["item_id"];
                        //必要
                        task->cabinet_id.first = j["data"]["cabinet_id"][0];
                        task->cabinet_id.second = j["data"]["cabinet_id"][1];
                    }
                    else if (task->task_type=="item_outbound")
                    {
                        //非必要
                        task->item_id = j["data"]["item_id"];
                        task->cabinet_id.first = j["data"]["cabinet_id"][0];
                        task->cabinet_id.second = j["data"]["cabinet_id"][1];
                    }
                }
                else if (task->action=="pick_up")
                {
                    // 解析 data 里面的数据
                    task->task_type = j["data"]["task_type"];
                    if (task->task_type=="item_store")
                    {
                        // 非必要
                        task->cabinet_id.first = j["data"]["cabinet_id"][0];
                        task->cabinet_id.second = j["data"]["cabinet_id"][1];
                        //必要
                        task->item_id = j["data"]["item_id"];
                    }
                    else if (task->task_type=="item_outbound")
                    {
                        //必要
                        task->item_id = j["data"]["item_id"];
                        task->cabinet_id.first = j["data"]["cabinet_id"][0];
                        task->cabinet_id.second = j["data"]["cabinet_id"][1];
                    }
                }
                else if (task->action=="open_gripper" || task->action=="close_gripper"){}
                else
                    return;
            }
            else
            {
                sendTcpMsg(400,"code值非法,不接受任务",task);
                return;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"json字段中的data错误: %s",e.what());
            sendTcpMsg(400,"json字段中的data错误: "+std::string(e.what()),task);
            return;
        }

        /* 执行任务 */
        bool expected = false;
        if (is_running.compare_exchange_strong(expected, true)) // compare_exchange_strong是线程安全的
        {
            std::thread(&FairinoRos2sdkControl::fairino_move_thread, this, task).detach();
        }else
        {
            RCLCPP_WARN(this->get_logger(),"正在执行任务，忽略此任务！");
            sendTcpMsg(400,"正在执行任务，忽略此任务！",task);
        }
    }
    void fairino_move_thread(const std::shared_ptr<TaskData> task)
    {
        struct StateGuard {
            std::atomic<bool>& flag;
            StateGuard(std::atomic<bool>& f) : flag(f) { flag = true; }
            ~StateGuard() { flag = false; }
        } state_guard(is_running);

        /* 任务序列 */
        try
        {
            if (task->task_type=="item_store")
            {
                activate_gripper(1,0);
                RCLCPP_INFO(this->get_logger(),"物品入柜");
                if (task->action=="pick_up")
                {
                    int task_id=0;
                    RCLCPP_INFO(this->get_logger(), "正在执行抓取物品");
                    // 清除错误状态
                    if (!reset_all_error(task))
                        return;
                    // 去到小车
                    if (!MoveJ(++task_id,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 打开夹爪
                    if (!ControlGripper(task,1,100))
                        return;
                    // 等待物品到来
                    std::vector<double> item_pose = WaitItem(task);
                    if (item_pose.empty())
                        return;
                    // 机械臂下降（运动到物品前方）
                    if (!MoveL(++task_id,task,item_above_car_point[0],item_above_car_point[1],(item_above_car_point[2]+item_pose[2])/3,car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 机械臂向前（运动到物品上方）
                    if (!MoveL(++task_id,task,item_pose[0],item_pose[1],(item_above_car_point[2]+item_pose[2])/3,car_point_posture[0],car_point_posture[1],item_pose[5],speed))
                        return;
                    // 夹爪运动到物品（机械臂向下移）
                    if (!MoveL(++task_id,task,item_pose[0],item_pose[1],item_pose[2],car_point_posture[0],car_point_posture[1],item_pose[5],speed))
                        return;
                    // 关闭夹爪
                    if (!ControlGripper(task,1,0))
                        return;
                    // 回到小车
                    if (!MoveJ(++task_id,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 所有任务都执行成功，返回task_success
                    TaskSuccess(task);
                }
                else if (task->action=="place")
                {
                    int task_id=0;
                    RCLCPP_INFO(this->get_logger(),"正在执行放置物品");
                    // 清除错误状态
                    if (!reset_all_error(task))
                        return;
                    // 去到柜子前
                    if (!MoveJ(++task_id,task,cabinet_front_points[task->cabinet_id.second][0],cabinet_front_points[task->cabinet_id.second][1],cabinet_front_points[task->cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2],speed))
                        return;
                    // 移动到柜子里面（放置物品）【中间点】
                    // if (!MoveJ(++task_id,action,task,cabinet_inside_points[cabinet_id.second][0]-50,cabinet_inside_points[cabinet_id.second][1],cabinet_inside_points[cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2],speed))
                    //     return;
                    // 移动到柜子里面（放置物品）
                    if (!MoveL(++task_id,task,cabinet_inside_points[task->cabinet_id.second][0],cabinet_inside_points[task->cabinet_id.second][1],cabinet_inside_points[task->cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2],speed))
                        return;
                    // 打开夹爪
                    if (!ControlGripper(task,1,100))
                        return;
                    // 移动到柜子里面（放置物品）【中间点】
                    // if (!MoveJ(++task_id,action,task,cabinet_inside_points[cabinet_id.second][0]-50,cabinet_inside_points[cabinet_id.second][1],cabinet_inside_points[cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2],speed))
                    //     return;
                    // 出去柜子
                    if (!MoveL(++task_id,task,cabinet_front_points[task->cabinet_id.second][0],cabinet_front_points[task->cabinet_id.second][1],cabinet_front_points[task->cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2],speed))
                        return;
                    // 去到小车上
                    if (!MoveJ(++task_id,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 所有任务都执行成功，返回task_success
                    TaskSuccess(task);
                }
            }
            else if (task->task_type=="item_outbound")
            {
                activate_gripper(1,0);
                RCLCPP_INFO(this->get_logger(),"物品出柜");
                if (task->action=="pick_up")
                {
                    int task_id=0;
                    RCLCPP_INFO(this->get_logger(),"正在执行抓取物品");
                    // 清除错误状态
                    if (!reset_all_error(task))
                        return;
                    // 打开夹爪
                    if (!ControlGripper(task,1,100))
                        return;
                    // 去到物品识别点位
                    if (!MoveJ(++task_id,task,cabinet_recognize_points[task->cabinet_id.second][0],cabinet_recognize_points[task->cabinet_id.second][1],cabinet_recognize_points[task->cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2],speed))
                        return;
                    //等待物品
                    std::vector<double> item_pose = WaitItem(task);
                    if (item_pose.empty())
                        return;
                    // 移动到物品前面
                    if (!MoveL(++task_id,task,item_pose[0],cabinet_recognize_points[task->cabinet_id.second][1],item_pose[2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2],speed))
                        return;
                    // 移动到物品
                    if (!MoveL(++task_id,task,item_pose[0],item_pose[1],item_pose[2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2],speed))
                        return;
                    // 关闭夹爪
                    if (!ControlGripper(task,1,0))
                        return;
                    // 离开柜子
                    if (!MoveL(++task_id,task,cabinet_front_points[task->cabinet_id.second][0],cabinet_front_points[task->cabinet_id.second][1],cabinet_front_points[task->cabinet_id.second][2],cabinet_point_posture[0],cabinet_point_posture[1],cabinet_point_posture[2],speed))
                        return;
                    // 去到小车上
                    if (!MoveJ(++task_id,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 所有任务都执行成功，返回task_success
                    TaskSuccess(task);
                }
                else if (task->action=="place")
                {
                    int task_id=0;
                    RCLCPP_INFO(this->get_logger(),"正在执行放置物品");
                    // 清除错误状态
                    if (!reset_all_error(task))
                        return;
                    // 去到小车上
                    if (!MoveJ(++task_id,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 机械臂下降
                    if (!MoveL(++task_id,task,item_above_car_point[0],item_above_car_point[1],(item_above_car_point[2]+item_in_car_point[2])/3,car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 机械臂向前（运动到物品前方）
                    if (!MoveL(++task_id,task,item_in_car_point[0],item_in_car_point[1],(item_above_car_point[2]+item_in_car_point[2])/3,car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 把物品放到小车正上方（夹爪下降）
                    if (!MoveL(++task_id,task,item_in_car_point[0],item_in_car_point[1],item_in_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 打开夹爪
                    if (!ControlGripper(task,1,100))
                        return;
                    // 去到小车上
                    if (!MoveJ(++task_id,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed))
                        return;
                    // 所有任务都执行成功，返回task_success
                    TaskSuccess(task);
                }
            }
            else if (task->action=="open_gripper")
            {
                RCLCPP_INFO(this->get_logger(),"正在演示demo：打开夹爪");
                activate_gripper(1,100);
                TaskSuccess(task);
            }
            else if (task->action=="close_gripper")
            {
                RCLCPP_INFO(this->get_logger(),"正在演示demo：闭合夹爪");
                activate_gripper(1,0);
                TaskSuccess(task);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "线程异常: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "线程未知异常");
        }

    }

    // 获取任务状态
    void get_task_status(const fairino_msgs::msg::RobotNonrtState& msg)
    {
        machineryTCP_current_pose[0] = msg.cart_x_cur_pos;
        machineryTCP_current_pose[1] = msg.cart_y_cur_pos;
        machineryTCP_current_pose[2] = msg.cart_z_cur_pos;
        machineryTCP_current_pose[3] = msg.cart_a_cur_pos;
        machineryTCP_current_pose[4] = msg.cart_b_cur_pos;
        machineryTCP_current_pose[5] = msg.cart_c_cur_pos;

        last_status.store(this_status.load());
        this_status.store(msg.robot_motion_done & msg.grip_motion_done);
        if (last_status.load()==0 && this_status.load()==1)
            machine_status.store(SeqState::IDLE);
        else if (last_status.load()==1 && this_status.load()==0)
            machine_status.store(SeqState::BUSY);

        // DEBUG信息
        // if (machine_status.load()==SeqState::IDLE)
        //     RCLCPP_INFO(this->get_logger(),"任务状态：IDLE");
        // else if (machine_status.load()==SeqState::BUSY)
        //     RCLCPP_INFO(this->get_logger(),"任务状态：BUSY");
    }

    // 获取要抓取的物品 id 和 位姿
    void item_pose_callback(const geometry_msgs::msg::PoseStamped& item_pose)
    {
        std::lock_guard<std::mutex> lock(item_pose_mutex_);
        this->item_pose.header = item_pose.header;
        this->item_pose.pose = item_pose.pose;
    }
    void item_id_callback(const std_msgs::msg::Int32MultiArray& msg)
    {
        if (!msg.data.empty())
            if (item_id!=msg.data[0])
            {
                std::lock_guard<std::mutex> lock(item_id_mutex_);
                item_id = msg.data[0];
            }
    }

    // 发送TCP消息（工具函数）
    void sendTcpMsg(const int& code,const std::string& message,const std::shared_ptr<TaskData>& task)
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
            {"action", task->action},
            {"message", message},
            {"datetime", time_str},
            {"data",{{"task_type",task->task_type},{"item_id",item_id},{"cabinet_id",{task->cabinet_id.first,task->cabinet_id.second}}}}
        }).dump();
        tcp_msg_pub_->publish(str);
    }

    // 比较当前位姿和指定位姿（工具函数）
    bool compare_machinery_pose(const double& x, const double& y,const double& z,const double& rx,const double& ry,const double& rz,const double& pos_threshold_mm = 5.0,const double& ori_threshold_deg = 3.0)
    {
        // 比较位置 (欧氏距离)
        double pos_diff = std::sqrt(
            std::pow(machineryTCP_current_pose[0] - x, 2) +
            std::pow(machineryTCP_current_pose[1] - y, 2) +
            std::pow(machineryTCP_current_pose[2] - z, 2)
        );

        if (pos_diff > pos_threshold_mm) {
            RCLCPP_WARN(this->get_logger(), "位置偏差过大: %f mm", pos_diff);
            return false;
        }

        // 3. 比较姿态 (RX, RY, RZ) - 处理 ±180 度跳变问题
        double diff = machineryTCP_current_pose[3] - rx;
        // 将误差归一化到 [-180, 180] 区间
        while (diff > 180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;
        if (std::abs(diff) > ori_threshold_deg) {
            RCLCPP_WARN(this->get_logger(), "角度偏差过大 (轴x): %f 度",diff);
            return false;
        }

        diff = machineryTCP_current_pose[4] - ry;
        // 将误差归一化到 [-180, 180] 区间
        while (diff > 180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;
        if (std::abs(diff) > ori_threshold_deg) {
            RCLCPP_WARN(this->get_logger(), "角度偏差过大 (轴y): %f 度", diff);
            return false;
        }

        diff = machineryTCP_current_pose[5] - rz;
        // 将误差归一化到 [-180, 180] 区间
        while (diff > 180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;
        if (std::abs(diff) > ori_threshold_deg) {
            RCLCPP_WARN(this->get_logger(), "角度偏差过大 (轴z): %f 度", diff);
            return false;
        }

        return true;
    }

    /* 任务序列函数 */
    // 基础设置
    void basic_setting()
    {
        // 等待服务端启动
        while (!fairino_control_client->service_is_ready() && rclcpp::ok()) {
            RCLCPP_WARN(this->get_logger(), "正在等待法奥机械臂服务端启动中...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // 使能
        std::string temp = "1";
        auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
        request->cmd_str = "RobotEnable(1)";
        auto future = fairino_control_client->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            auto result = future.get();
            temp = result->cmd_res;
            if (temp=="0")
                RCLCPP_INFO(this->get_logger(), "【BasicSetting】服务调用成功，输出结果：%s", temp.data());
            else
                RCLCPP_INFO(this->get_logger(), "【BasicSetting】服务调用失败，输出结果：%s", temp.data());
        } else {
            RCLCPP_ERROR(this->get_logger(), "【BasicSetting】服务调用超时或失败");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 设置手动模式
        request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
        request->cmd_str = "Mode(1)";
        future = fairino_control_client->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            auto result = future.get();
            temp = result->cmd_res;
            if (temp =="0")
                RCLCPP_INFO(this->get_logger(), "【BasicSetting】服务调用成功，输出结果：%s", temp.data());
            else
                RCLCPP_INFO(this->get_logger(), "【BasicSetting】服务调用失败，输出结果：%s", temp.data());
        } else {
            RCLCPP_ERROR(this->get_logger(), "【BasicSetting】服务调用超时或失败");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 激活夹爪
    void activate_gripper(const int& gripper_id,const int& open_degree)
    {
        // 初始化夹爪（夹爪张开）
        auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
        request->cmd_str = "MoveGripper("+std::to_string(gripper_id)+","+std::to_string(open_degree)+")";
        auto future = fairino_control_client->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            auto result = future.get();
            std::string temp=result->cmd_res;
            if (temp=="0")
            {
                RCLCPP_INFO(this->get_logger(),"【ActivateGripper】服务调用成功：%s",temp.data());
            }
            else
                RCLCPP_ERROR(this->get_logger(),"【ActivateGripper】服务调用失败：%s，请重新插拔夹爪线",temp.data());
        } else
            RCLCPP_ERROR(this->get_logger(), "【ActivateGripper】服务调用失败，请重新插拔夹爪线");
    }

    // 清除错误状态
    bool reset_all_error(const std::shared_ptr<TaskData>& task)
    {
        int try_count = 1;
        std::string temp = "1";
        while ( temp != "0"&& rclcpp::ok())
        {
            if (machine_status.load()==SeqState::IDLE)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "ResetAllError()";
                auto future = fairino_control_client->async_send_request(request);
                if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
                {
                    auto result = future.get();
                    temp = result->cmd_res;
                    RCLCPP_INFO(this->get_logger(), "【ResetAllError】服务调用成功：%s", temp.data());
                    if (temp == "0")
                        break;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【ResetAllError】服务调用超时或失败");
                }
                try_count++;
            }
            if (try_count>=2)
            {
                RCLCPP_ERROR(this->get_logger(),"【ResetAllError】超过最大尝试次数，任务失败！");
                sendTcpMsg(400,"【ResetAllError】超过最大尝试次数，任务失败！",task);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        return true;
    }

    // 笛卡尔积坐标走点
    bool MoveJ(const int& task_id, const std::shared_ptr<TaskData>& task, const double& x, const double& y, const double& z, const double& rx, const double& ry, const double& rz, const int& speed)
    {
        while (rclcpp::ok())
        {
            // 只有在机械臂空闲时才能进入
            if (machine_status.load()==SeqState::IDLE)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+","+std::to_string(rx)+","+std::to_string(ry)+","+std::to_string(rz)+")";
                auto future = fairino_control_client->async_send_request(request);
                if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
                {
                    auto result = future.get();
                    std::string temp = result->cmd_res;
                    if (temp=="0")
                        RCLCPP_INFO(this->get_logger(),"【MoveJ】服务调用成功：%s",temp.data());
                    else
                    {
                        RCLCPP_INFO(this->get_logger(),"【MoveJ】服务调用失败，错误码：%s",temp.data());
                        sendTcpMsg(400,"【MoveJ】服务调用失败！请检查点位是否合法",task);
                        return false;
                    }
                }else {
                    RCLCPP_ERROR(this->get_logger(), "【MoveJ】服务调用失败");
                    sendTcpMsg(400,"【MoveJ】服务调用失败！请检查点位是否合法",task);
                    return false;
                }

                int try_count = 1;
                request->cmd_str = "MoveJ(CART"+std::to_string(task_id)+","+std::to_string(speed)+",1,0,0,0,0,0)";
                while (rclcpp::ok())
                {
                    future = fairino_control_client->async_send_request(request);
                    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
                    {
                        auto result = future.get();
                        std::string temp=result->cmd_res;
                        RCLCPP_INFO(this->get_logger(),"【MoveJ】服务调用成功：%s",temp.data());
                        if (temp=="0")
                        {
                            RCLCPP_INFO(this->get_logger(),"【MoveJ】服务调用成功：%s",temp.data());
                            if (!compare_machinery_pose(x,y,z,rx,ry,rz))
                            {
                                machine_status.store(SeqState::BUSY);
                                break;
                            }
                            else
                                return true;
                        }
                        reset_all_error(task);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "【MoveJ】服务调用失败");
                    }

                    if (try_count++ >=2)
                    {
                        sendTcpMsg(400,"【MoveJ】超过最大尝试次数，任务失败！",task);
                        if (!compare_machinery_pose(item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2]))
                            MoveJ(1,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed); // 回到小车上
                        return false;
                    }
                }

                while (machine_status.load()!=SeqState::IDLE && rclcpp::ok())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        return true;
    }

    // 笛卡尔积直线
    bool MoveL(const int& task_id, const std::shared_ptr<TaskData>& task,const double& x, const double& y ,const double& z, const double& rx, const double& ry, const double& rz, const int& speed)
    {
        while (rclcpp::ok())
        {
            if (machine_status.load()==SeqState::IDLE)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "CARTPoint("+std::to_string(task_id)+","+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+","+std::to_string(rx)+","+std::to_string(ry)+","+std::to_string(rz)+")";
                auto future = fairino_control_client->async_send_request(request);
                if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
                {
                    auto result = future.get();
                    std::string temp= result->cmd_res;
                    if (temp=="0")
                        RCLCPP_INFO(this->get_logger(),"【MoveL】服务调用成功：%s",temp.data());
                    else
                    {
                        RCLCPP_INFO(this->get_logger(),"【MoveL】服务调用失败，错误码：%s",temp.data());
                        sendTcpMsg(400,"【MoveL】服务调用失败！请检查点位是否合法",task);
                        return false;
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【MoveL】服务调用失败");
                    sendTcpMsg(400,"【MoveL】服务调用失败！请检查点位是否合法",task);
                    return false;
                }

                int try_count=1;
                while (rclcpp::ok())
                {
                    request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                    request->cmd_str = "MoveL(CART"+std::to_string(task_id)+","+std::to_string(speed)+",1,0,0,0,0,0)";
                    future = fairino_control_client->async_send_request(request);
                    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
                    {
                        auto result = future.get();
                        std::string temp= result->cmd_res;
                        if (temp=="0")
                        {
                            RCLCPP_INFO(this->get_logger(),"【MoveL】服务调用成功：%s",temp.data());
                            if (!compare_machinery_pose(x,y,z,rx,ry,rz))
                            {
                                machine_status.store(SeqState::BUSY);
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                                break;
                            }
                            else
                                return true;
                        }
                        reset_all_error(task);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "【MoveL】服务调用失败");
                    }
                    try_count++;
                    if (try_count>=2)
                    {
                        RCLCPP_ERROR(this->get_logger(),"超过最大尝试次数，任务失败！");
                        sendTcpMsg(400,"超过最大尝试次数，任务失败！",task);
                        // MoveJ(1,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed); // 回到小车上
                        return false;
                    }
                }

                // 空闲时才能退出
                while (machine_status.load()!=SeqState::IDLE && rclcpp::ok())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // 夹爪控制
    bool ControlGripper(const std::shared_ptr<TaskData>& task, const int& gripper_id, const int& open_degree)
    {
        while (rclcpp::ok())
        {
            if (machine_status.load()==SeqState::IDLE)
            {
                auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                request->cmd_str = "MoveGripper("+std::to_string(gripper_id)+","+std::to_string(open_degree)+")";
                auto future = fairino_control_client->async_send_request(request);
                if (future.wait_for(std::chrono::seconds(5))==std::future_status::ready)
                {
                    auto result = future.get();
                    std::string temp=result->cmd_res;
                    if (temp=="0")
                    {
                        RCLCPP_INFO(this->get_logger(),"【夹爪】服务调用成功：%s",temp.data());
                        machine_status.store(SeqState::BUSY);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        break;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(),"【夹爪】服务调用失败：%s，请重新插拔夹爪线",temp.data());
                        sendTcpMsg(400,"夹爪服务调用失败："+temp+"，请重新插拔夹爪线",task);
                        return false;
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "【夹爪】服务调用失败，请重新插拔夹爪线");
                    sendTcpMsg(400,"夹爪服务调用失败，请尝试重启机械臂和工控机。若尝试多次还是不行，请联系工程师！",task);
                    return false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // 空闲时才能退出
        while (machine_status.load()!=SeqState::IDLE && rclcpp::ok())
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return true;
    }

    // 等待物品
    std::vector<double> WaitItem(const std::shared_ptr<TaskData>& task)
    {
        RCLCPP_WARN(this->get_logger(),"等待物品");
        rclcpp::Time past_time = this->get_clock()->now();
        while (rclcpp::ok())
        {
            if (machine_status.load()==SeqState::IDLE)
            {
                // 获取物品id和pose
                geometry_msgs::msg::PoseStamped current_pose;
                long current_id;
                {
                    std::scoped_lock lock(item_pose_mutex_, item_id_mutex_);
                    current_pose = this->item_pose;
                    current_id = this->item_id;
                }

                // 时间戳校验（这里是 传输延时+相机处理延时）
                if (std::abs((this->get_clock()->now() - current_pose.header.stamp).seconds()) > 1.0)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "尚未收到物品姿态消息");
                }
                // 物品id校验
                else if (task->item_id!=current_id)
                {
                    RCLCPP_ERROR(this->get_logger(),"物品ID不匹配，待抓取ID为%ld，实际物品ID为%ld！取消执行任务",task->item_id,current_id);
                    sendTcpMsg(400,"物品ID不匹配，待抓取ID为"+std::to_string(task->item_id)+"，实际物品ID为"+std::to_string(current_id)+"！取消执行任务",task);
                    MoveJ(1,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed);    // 去到小车位置【复位】
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
                RCLCPP_WARN(this->get_logger(),"机械臂正在执行任务");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                past_time = this->get_clock()->now();
            }

            // 等待物品超时
            if ((this->get_clock()->now()-past_time).seconds()>=30)
            {
                RCLCPP_ERROR(this->get_logger(),"等待物品超时，超时时间为30s");
                sendTcpMsg(400,"等待物品超时",task);
                MoveJ(1,task,item_above_car_point[0],item_above_car_point[1],item_above_car_point[2],car_point_posture[0],car_point_posture[1],car_point_posture[2],speed);    // 去到小车位置【复位】
                return {};
            }
        }
    }

    // 任务完成
    void TaskSuccess(const std::shared_ptr<TaskData>& task)
    {
        while (rclcpp::ok())
        {
            if (machine_status.load()==SeqState::IDLE)
            {
                RCLCPP_INFO(this->get_logger(),"任务执行成功");
                sendTcpMsg(200,"任务执行成功",task);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // 定义阶段状态机，用于判断上升和下降沿
    enum class SeqState
    {
        BUSY,
        IDLE
    };

    // 机械臂点位和姿态（从参数服务器获取）
    int speed;                                                  // 机械臂的速度
    std::vector<double> item_above_car_point;                   // 物体在车上方的抓取点位（原点）
    std::vector<double> item_in_car_point;                      // 物体在车上的放置点
    std::vector<double> car_point_posture;                      // 原点时夹爪的姿态
    std::vector<std::vector<double>> cabinet_front_points;      // 机械臂到达柜子前面点位
    std::vector<std::vector<double>> cabinet_inside_points;     // 进入柜子放置物品点位
    std::vector<std::vector<double>> cabinet_recognize_points;  // 在柜子前识别物品点位
    std::vector<double> cabinet_point_posture;                  // 在柜子的夹爪角度
    std::vector<double> desired_gripper_rotation_x;             // 预期夹爪姿态

    // 状态相关
    std::atomic<int> last_status;                   // 上一次从机器人 / SDK 读取到的状态
    std::atomic<int> this_status;                   // 当前机器人 / SDK 状态
    std::atomic<SeqState> machine_status;           // 核心状态机状态（FSM 当前节点）
    std::atomic<bool> is_running;                   // 是否正在执行一条完整流程（防重入）
    std::vector<double> machineryTCP_current_pose;  // 当前TCP（夹爪）位姿

    // TCP 通信
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tcp_msg_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tcp_msg_pub_;

    // Fairino 机器人状态 & 控制接口
    rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr fairino_nonrt_state_data_sub_;
    std::shared_ptr<rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>> fairino_control_client;
    rclcpp::TimerBase::SharedPtr startup_init_timer_;

    // 物品ID与姿态获取
    int item_id;                                // 当前物品ID
    geometry_msgs::msg::PoseStamped item_pose;  // 当前物品位姿
    std::mutex item_id_mutex_;                  // 物品ID锁
    std::mutex item_pose_mutex_;                // 物品位姿锁
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr item_pose_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr item_id_sub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FairinoRos2sdkControl>();

    // 使用多线程执行器，允许同时处理订阅数据和服务响应
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(node->client_node_);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
