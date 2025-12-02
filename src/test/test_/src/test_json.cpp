#include <nlohmann/json.hpp>
#include <iostream>

int main()
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

    std::string json_str = (nlohmann::json{
        {"code",200},
        {"action", "aaa"},
        {"message", "你好"},
        {"datetime", time_str},
        {"data",nlohmann::json::object()}
    }).dump();

    std::cout <<json_str<<std::endl;
}