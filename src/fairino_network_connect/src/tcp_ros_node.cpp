#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <nlohmann/json.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <iostream>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>

class TcpJsonClientNode : public rclcpp::Node
{
public:
    TcpJsonClientNode()
        : Node("tcp_json_client_node"),
          connected_(false)
    {
        device_id_   = declare_parameter<std::string>("deviceId", "000000");
        device_name_ = declare_parameter<std::string>("deviceName", "robotic_arm_01");
        server_ip_   = declare_parameter<std::string>("server_ip", "127.0.0.1");
        server_port_ = declare_parameter<int>("server_port", 9000);
        heartbeat_time_   = declare_parameter<int>("heartbeat_interval", 30);

        rclcpp::QoS keepLast_qos(rclcpp::KeepLast(100));
        keepLast_qos.reliable();

        ros_pub_ = create_publisher<std_msgs::msg::String>("tcp_to_ros_cmd", keepLast_qos);

        ros_sub_ = create_subscription<std_msgs::msg::String>(
            "ros_to_tcp_cmd",
            keepLast_qos,
            std::bind(&TcpJsonClientNode::rosMessageCallback, this, std::placeholders::_1)
        );

        tcp_thread_ = std::thread(&TcpJsonClientNode::tcpThread, this);

        heartbeat_timer_ = create_wall_timer(
            std::chrono::seconds(heartbeat_time_),
            std::bind(&TcpJsonClientNode::sendHeartbeat, this)
        );

        RCLCPP_INFO(get_logger(), "TCP JSON Client Node 已启动");
    }

    ~TcpJsonClientNode()
    {
        connected_ = false;
        if (tcp_thread_.joinable())
            tcp_thread_.join();
        close(sock_);
    }

private:

    // -------------ROS → TCP-------------
    void rosMessageCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            nlohmann::json j = nlohmann::json::parse(msg->data);

            j["deviceId"]   = device_id_;
            j["deviceName"] = device_name_;
            j["type"]       = 1;

            std::string out = j.dump();

            // 缓存发送的消息（用于重传）
            {
                std::lock_guard<std::mutex> lock(last_msg_mutex_);
                last_sent_msg_ = out;
            }

            sendToTcp(out);
        }
        catch (...) {
            RCLCPP_ERROR(get_logger(), "ROS → TCP JSON 解析失败");
        }
    }


    // -------------TCP → ROS：处理单条完整的 JSON-------------
    void processTcpMessage(const std::string &msg)
    {
        if (msg.empty()) return; // 忽略空行

        try {
            nlohmann::json j = nlohmann::json::parse(msg);

            // 1. 优先处理错误重传 (code: 400)
            if (j.contains("code") && j["code"] == 400) {
                // 打印服务器返回的具体错误信息，方便调试
                std::string err_msg = j.contains("message") ? j["message"].get<std::string>() : "unknown";
                RCLCPP_WARN(get_logger(), "收到服务器报错 (code:400): %s. 正在重传...", err_msg.c_str());
                
                resendLastMessage();
                return; 
            }

            // 2. 正常业务逻辑
            if (j.contains("deviceId") && j["deviceId"].get<std::string>() == device_id_) {
                RCLCPP_INFO(get_logger(), "收到指令，下发 ROS");
                j.erase("deviceId");
                j.erase("deviceName");
                j.erase("type");

                std_msgs::msg::String ros_msg;
                ros_msg.data = j.dump();
                ros_pub_->publish(ros_msg);
            }

        } catch (const std::exception &e) {
            // 这里打印 raw msg，如果再次解析失败，可以看到是哪一段数据出了问题
            RCLCPP_ERROR(get_logger(), "JSON 解析异常: %s | 原始内容: %s", e.what(), msg.c_str());
        }
    }

    void resendLastMessage()
    {
        std::string msg_to_send;
        {
            std::lock_guard<std::mutex> lock(last_msg_mutex_);
            if (last_sent_msg_.empty()) return;
            msg_to_send = last_sent_msg_;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 稍作延时
        sendToTcp(msg_to_send);
    }


    // -------------TCP 线程 (核心修改：处理粘包)-------------
    void tcpThread()
    {
        while (rclcpp::ok()) {
            if (!connectToServer()) {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                continue;
            }

            // 接收缓冲区
            char buffer[4096];
            
            // 清空之前的残留数据
            rx_buffer_.clear(); 

            while (connected_ && rclcpp::ok()) {
                ssize_t len = recv(sock_, buffer, sizeof(buffer), 0);
                
                if (len > 0) {
                    // 1. 将收到的原始数据追加到 string buffer 中
                    rx_buffer_.append(buffer, len);

                    // 2. 循环处理 buffer 中的每一行 (以 \n 分隔)
                    size_t pos = 0;
                    while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
                        // 提取一行完整的 JSON
                        std::string single_msg = rx_buffer_.substr(0, pos);
                        
                        // 从 buffer 中移除已提取的部分 (包括 \n)
                        rx_buffer_.erase(0, pos + 1);

                        // 处理这一条消息
                        if (!single_msg.empty()) {
                            processTcpMessage(single_msg);
                        }
                    }
                    // 此时 rx_buffer_ 中可能残留半条消息（没有 \n 结尾），
                    // 等待下一次 recv 补全它。
                } 
                else if (len == 0) {
                    RCLCPP_WARN(get_logger(), "服务器断开连接");
                    connected_ = false;
                    close(sock_);
                    break;
                }
                else {
                    RCLCPP_ERROR(get_logger(), "recv 错误");
                    connected_ = false;
                    close(sock_);
                    break;
                }
            }
        }
    }

    bool connectToServer()
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ < 0) return false;

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(server_port_);
        inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr);

        if (connect(sock_, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            close(sock_);
            return false;
        }

        RCLCPP_INFO(get_logger(), "TCP 连接成功");
        connected_ = true;
        
        // 可以在这里发个心跳或注册包，但要注意锁的问题
        // 这里简单返回，让 timer 去发心跳
        return true;
    }

    void sendToTcp(const std::string &msg)
    {
        if (!connected_) return;
        std::lock_guard<std::mutex> lock(socket_mutex_);
        std::string data = msg + "\n"; // 确保发送带换行符
        send(sock_, data.c_str(), data.size(), 0);
    }

    void sendHeartbeat()
    {
        if (!connected_) return;
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_local = *std::localtime(&t);
        char buf[64];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_local);
        std::string timestamp(buf);

        nlohmann::json heartbeat;
        heartbeat["deviceId"]   = device_id_;
        heartbeat["deviceName"] = device_name_;
        heartbeat["type"]       = 1;
        heartbeat["action"]     = "heartbeat";
        heartbeat["timestamp"]  = timestamp;
        heartbeat["message"]    = "心跳";

        sendToTcp(heartbeat.dump());
    }

    //-------------成员变量-------------
    std::string device_id_;
    std::string device_name_;
    std::string server_ip_;
    int server_port_;
    int heartbeat_time_;
    int sock_;
    std::atomic<bool> connected_;
    std::thread tcp_thread_;
    std::mutex socket_mutex_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_sub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    std::string last_sent_msg_;
    std::mutex last_msg_mutex_;
    
    //  新增: 接收缓冲区，用于解决粘包问题
    std::string rx_buffer_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpJsonClientNode>());
    rclcpp::shutdown();
    return 0;
}