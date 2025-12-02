#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>

#include <nlohmann/json.hpp>
#include <curl/curl.h>

#include <chrono>
#include <string>
#include <mutex>
#include <thread>
#include <filesystem>

// ----------工具函数：生成时间戳字符串----------
std::string getTimestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_local = *std::localtime(&t);

    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H_%M_%S", &tm_local);
    return std::string(buf);
}

// ----------libcurl 上传文件类----------
class NetworkManager {
public:
    bool uploadFileAsync(const std::string& filepath, const std::string& url, const std::string& deviceId)
    {
        std::thread([filepath, url, deviceId]() {
            CURL *curl = curl_easy_init();
            if (!curl) {
                RCLCPP_ERROR(rclcpp::get_logger("NetworkManager"), "curl 初始化失败");
                return;
            }

            curl_mime *form = curl_mime_init(curl);

            // 文件字段
            curl_mimepart *file_field = curl_mime_addpart(form);
            curl_mime_name(file_field, "file");
            curl_mime_filedata(file_field, filepath.c_str());

            // deviceName 字段
            curl_mimepart *dev_field = curl_mime_addpart(form);
            curl_mime_name(dev_field, "deviceId");
            curl_mime_data(dev_field, deviceId.c_str(), CURL_ZERO_TERMINATED);

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                RCLCPP_ERROR(rclcpp::get_logger("NetworkManager"), "上传失败: %s", curl_easy_strerror(res));
            } else {
                RCLCPP_INFO(rclcpp::get_logger("NetworkManager"), "上传成功: %s", filepath.c_str());
            }

            curl_mime_free(form);
            curl_easy_cleanup(curl);
        }).detach();

        return true;
    }
};

// ----------主相机控制节点----------
class CameraControlNode : public rclcpp::Node {
public:
    CameraControlNode()
        : Node("camera_control_node"),
          is_recording_(false)
    {
        declare_parameter<std::string>("save_directory", "out/camera_output");
        declare_parameter<std::string>("upload_url", "http://127.0.0.1:8000/upload");
        declare_parameter<std::string>("deviceId", "1990235585032663041");

        get_parameter("save_directory", save_directory_);
        get_parameter("upload_url", upload_url_);
        get_parameter("deviceId", device_Id_);

        std::filesystem::create_directories(save_directory_);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "camera/color/image_raw", 10,
            std::bind(&CameraControlNode::imageCallback, this, std::placeholders::_1)
        );

        command_sub_ = create_subscription<std_msgs::msg::String>(
            "tcp_to_ros_cmd", 10,
            std::bind(&CameraControlNode::commandCallback, this, std::placeholders::_1)
        );

        camera_result_pub_ = create_publisher<std_msgs::msg::String>(
            "ros_to_tcp_cmd", 10
        );

        network_manager_ = std::make_shared<NetworkManager>();

        RCLCPP_INFO(get_logger(), "Camera Control Node 已启动。");
    }

private:

// -------------命令回调函数-------------
    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        nlohmann::json j;
        try {
            j = nlohmann::json::parse(msg->data);
        } catch (...) {
            publishResult("unknown", "解析JSON失败", 400);
            return;
        }

        std::string cmd = j.value("action", "");

        if (cmd == "take_photos") {
            takePhoto();
        }
        else if (cmd == "video_record") {
            startRecording();
        }
    }

// -------------图像流回调函数（处理相机发布的图像流）-------------
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (...) {
            return;
        }
        std::lock_guard<std::mutex> lock(frame_mutex_);
        cv_ptr->image.copyTo(latest_frame_);

        if (is_recording_ && video_writer_.isOpened()) {
            video_writer_.write(cv_ptr->image);
        }
    }

// -------------拍照-------------
    void takePhoto()
    {
        //摄像头在工作的话则不发生动作
        if (is_recording_) {
            publishResult("take_photos", "正在录制视频，无法拍照", 400);
            return;
        }

        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (latest_frame_.empty()) {
                publishResult("take_photos", "没有收到摄像头图像", 400);
                return;
            }
            frame = latest_frame_.clone();
        }

        std::string filename = "photo_" + getTimestamp() + ".png";
        std::string filepath = save_directory_ + "/" + filename;
        cv::imwrite(filepath, frame);

        RCLCPP_INFO(get_logger(), "照片已保存: %s", filepath.c_str());

        network_manager_->uploadFileAsync(filepath, upload_url_, device_Id_);

        publishResult("take_photos", "拍照", 200, filepath);
    }

// -------------开始录制-------------
    void startRecording()
    {   //摄像头在工作的话则不发生动作
        if (is_recording_) {
            publishResult("video_record", "已经在录制中", 400);
            return;
        }

        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (latest_frame_.empty()) {
                publishResult("video_record", "没有收到摄像头图像", 400);
                return;
            }
            frame = latest_frame_.clone();
        }

        int fps = 30;
        cv::Size size(frame.cols, frame.rows);

        current_video_filepath_ = save_directory_ + "/video_" + getTimestamp() + ".mp4";
        if (!video_writer_.open(current_video_filepath_, cv::VideoWriter::fourcc('h','2','6','4'), fps, size)) {
            publishResult("video_record", "VideoWriter打开失败", 400);
            return;
        }

        is_recording_ = true;
        record_start_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(), "开始录制: %s", current_video_filepath_.c_str());
        recording_timer_ = create_wall_timer(std::chrono::seconds(1),
            std::bind(&CameraControlNode::checkRecordingTimeout, this));
    }

// -------------检查时间（用于控制录制时间在30s左右）-------------
    void checkRecordingTimeout()
    {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - record_start_).count();
        if (elapsed >= 31)
            stopRecording();
    }

// -------------停止录制（相当于录制结束并上传、返回信息的逻辑）-------------
    void stopRecording()
    {
        if (!is_recording_) return;

        is_recording_ = false;
        if (recording_timer_) recording_timer_->cancel();
        video_writer_.release();

        network_manager_->uploadFileAsync(current_video_filepath_, upload_url_, device_Id_);
        publishResult("video_record", "录像", 200, current_video_filepath_);
    }

// -------------返回json信息-------------
    void publishResult(const std::string& action, const std::string& message, int code,
                       const std::string& file_path = "")
    {
        std_msgs::msg::String msg;
        nlohmann::json j;
        j["action"] = action;
        j["message"] = message;
        j["code"] = code;
        j["datetime"] = getTimestamp();

        if (!file_path.empty()) {
            std::string filename = std::filesystem::path(file_path).filename().string();
            j["data"]["filename"] =  filename;
        }

        msg.data = j.dump();
        camera_result_pub_->publish(msg);
    }

// -------------参数-------------
private:
    bool is_recording_;
    std::string save_directory_;
    std::string upload_url_;
    std::string device_Id_;

    cv::Mat latest_frame_;
    std::mutex frame_mutex_;

    cv::VideoWriter video_writer_;
    std::string current_video_filepath_;
    rclcpp::TimerBase::SharedPtr recording_timer_;

    std::chrono::steady_clock::time_point record_start_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_result_pub_;

    std::shared_ptr<NetworkManager> network_manager_;
};

// -------------main-------------
int main(int argc, char **argv)
{
    curl_global_init(CURL_GLOBAL_ALL);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraControlNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();

    curl_global_cleanup();
    return 0;
}
