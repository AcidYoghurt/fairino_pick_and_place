#include <curl/curl.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

class FairinoNetworkConnect : public rclcpp::Node
{
public:
    FairinoNetworkConnect() : rclcpp::Node("FairinoNetworkConnectNode")
    {
        // 参数
        this->declare_parameter("server_ip","127.0.0.1");
        server_ip = this->get_parameter("server_ip").as_string();
        this->declare_parameter("server_port",5000);
        server_port = this->get_parameter("server_port").as_int();

        // Pub和Sub
        Http2RosMsgPub = this->create_publisher<std_msgs::msg::String>("network/receiveMsg",10);
        Ros2HttpMsgSub = this->create_subscription<std_msgs::msg::String>("network/sendMsg",20,std::bind(&FairinoNetworkConnect::sendHttpMessage,this,std::placeholders::_1));

        // 启动监听线程
        listener_thread_ = std::thread(&FairinoNetworkConnect::listenHttpMessage, this);
    }

    ~FairinoNetworkConnect()
    {
        if(listener_thread_.joinable()) {
            listener_thread_.join();
        }
    }

private:
    // WriteCallback 改成直接发布
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
    {
        FairinoNetworkConnect* node = static_cast<FairinoNetworkConnect*>(userp);
        std::string data((char*)contents, size * nmemb);

        std_msgs::msg::String msg;
        msg.data = data;
        node->Http2RosMsgPub->publish(msg);

        RCLCPP_INFO(node->get_logger(), "收到HTTP数据: %s", data.c_str());
        return size * nmemb;
    }

    // 一直监听HTTP消息
    void listenHttpMessage()
    {
        while (rclcpp::ok()) {
            CURL* curl = curl_easy_init();
            if(curl) {
                std::string url = "http://" + server_ip + ":" + std::to_string(server_port);
                curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
                curl_easy_setopt(curl, CURLOPT_WRITEDATA, this);

                // 长连接，不超时
                curl_easy_setopt(curl, CURLOPT_TIMEOUT, 0L);
                curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

                RCLCPP_INFO(this->get_logger(), "开始HTTP监听: %s", url.c_str());
                CURLcode res = curl_easy_perform(curl);

                if(res != CURLE_OK) {
                    RCLCPP_ERROR(this->get_logger(), "HTTP监听失败: %s", curl_easy_strerror(res));
                    // 等待一段时间再重试
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
                curl_easy_cleanup(curl);
            }
        }
    }

    // 发送Http消息
    void sendHttpMessage(const std_msgs::msg::String& msg)
    {
        CURL* curl;
        CURLcode res;
        std::string message = msg.data;

        curl = curl_easy_init();
        if (curl)
        {
            std::string url = "http://" + server_ip + ":" + std::to_string(server_port);
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, message.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, message.size());
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);

            res = curl_easy_perform(curl);
            if(res == CURLE_OK) {
                RCLCPP_INFO(this->get_logger(), "发送HTTP消息成功: %s", message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "发送HTTP消息失败: %s", curl_easy_strerror(res));
            }
            curl_easy_cleanup(curl);
        }
    }

    std::string server_ip;
    int server_port;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Http2RosMsgPub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Ros2HttpMsgSub;
    std::thread listener_thread_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FairinoNetworkConnect>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
