#include <robot.h>
#include <robot_error.h>
#include <cstdint>
#include <cstdio>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

class TestFairinoCppSdk : public rclcpp::Node
{
public:
    TestFairinoCppSdk() : Node("TestRos2SdkNode")
    {
        // 机器人基础配置
        robot = FRRobot();
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        robot.RPC("192.168.58.2");
        robot.DragTeachSwitch(0);
        robot.RobotEnable(1);
        robot.SetReConnectParam(true, 30000, 500);
        RCLCPP_INFO(this->get_logger(),"初始化机器人成功");

        // Pub和Sub
        tcp_msg_pub_ = this->create_publisher<std_msgs::msg::String>("ros_to_tcp_cmd",10);
        tcp_msg_sub_ = this->create_subscription<std_msgs::msg::String>("tcp_to_ros_cmd",10,std::bind(&TestFairinoCppSdk::tcp_callback,this,std::placeholders::_1));
        item_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("aruco/pose_base", 10,std::bind(&TestFairinoCppSdk::item_pose_callback, this, std::placeholders::_1));
        item_id_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>("aruco/detected_markers",10,std::bind(&TestFairinoCppSdk::item_id_callback,this,std::placeholders::_1));
    }
private:
    // 执行任务序列
    void tcp_callback(std_msgs::msg::String msg)
    {
        if (!is_running)
        {
            std::thread(&TestFairinoCppSdk::fairino_move_thread, this, msg).detach();
        }else
        {
            RCLCPP_WARN(this->get_logger(),"正在执行任务，忽略此任务！");
            sendTcpMsg(400,"","正在执行任务，忽略此任务！");
        }
    }
    void fairino_move_thread(std_msgs::msg::String msg)
    {
        struct StateGuard {
            std::atomic<bool>& flag;
            StateGuard(std::atomic<bool>& f) : flag(f) { flag = true; }
            ~StateGuard() { flag = false; }
        } state_guard(is_running);

        /* 接收网络信息 */
        std::string task_type;
        std::string action;
        long item_id_net;
        std::pair<int,int> cabinet_id = std::pair<int,int>();
        // 解析json数据
        try
        {
            nlohmann::json j = nlohmann::json::parse(msg.data);
            if (j["code"]==200)
            {
                if (j["action"]=="place" || j["action"]=="pick_up")
                {
                    if ((j["data"]["task_type"]=="item_store" || j["data"]["task_type"]=="item_outbound"))
                    {
                        task_type = j["data"]["task_type"];
                        action = j["action"];
                        item_id_net = j["data"]["item_id"];
                        cabinet_id.first = j["data"]["cabinet_id"][0];
                        cabinet_id.second = j["data"]["cabinet_id"][1];
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(),"task_type信息错误！");
                        sendTcpMsg(400,action,"data中的task_type信息错误");
                        return;
                    }
                }
                else
                    return;
            }
            else
            {
                sendTcpMsg(400,action,"code为400,不接受任务");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"无法解析 网络消息 json数据：%s",e.what());
            sendTcpMsg(400,"","无法解析网络消息json数据: "+std::string(e.what()));
            return;
        }

        /* 任务序列 */
        if (task_type=="item_store")
        {
            RCLCPP_INFO(this->get_logger(),"物品入柜");
            if (action=="pick_up")
            {

            }
            else if (action=="place")
            {

            }
        }
        else if (task_type=="item_outbound")
        {
            RCLCPP_INFO(this->get_logger(),"物品出柜");
            if (action=="pick_up")
            {

            }
            if (action=="place")
            {

            }
        }
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

    FRRobot robot;
    std::atomic<bool> is_running;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tcp_msg_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tcp_msg_pub_;

    std::mutex item_id_mutex_;
    std::mutex item_pose_mutex_;
    int item_id;
    geometry_msgs::msg::PoseStamped item_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr item_pose_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr item_id_sub;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestFairinoCppSdk>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
