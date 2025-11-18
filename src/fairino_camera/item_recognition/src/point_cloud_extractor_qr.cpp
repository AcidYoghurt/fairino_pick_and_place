#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <fairino_msg/msg/qr_msg.hpp>
#include <fairino_msg/srv/item_msg.hpp>
#include <fairino_msg/msg/collision_object_manage.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

// 测试：
// ros2 service call /itemMsg_trigger fairino_msg/srv/ItemMsg

class PointCloudExtractor : public rclcpp::Node
{
public:
    PointCloudExtractor() : Node("point_cloud_extractor")
    {
        // 创建TF2监听器
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // 创建消息过滤器订阅器
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera/depth_rect_to_color/camera_info",
            rclcpp::QoS(10).reliable(),std::bind(&PointCloudExtractor::depth_camera_info_callback,this,std::placeholders::_1));
        item_center_point_sub_ = std::make_shared<message_filters::Subscriber<fairino_msg::msg::QrMsg>>(this, "qr/qr_msg");
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/depth_rect_to_color/image");
        // 使用近似时间同步策略
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *item_center_point_sub_, *depth_sub_, *aruco_single_sub_);
        sync_->registerCallback(std::bind(&PointCloudExtractor::syncCallback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3));

        // 二维码超时清空计时器
        qr_timeout_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),[this]()
        {
            if (qr_time_valid_==true)
            {
                if (((this->now() - last_qr_time_).seconds() > 0.5) && (item_msg.item_id.data != "" || item_msg.car_id.data != ""))
                {
                    RCLCPP_INFO(this->get_logger(),"二维码超时，清空 item_msg");
                    std::lock_guard<std::mutex> lock(latest_mutex_);
                    item_msg.item_id.data = "";
                    item_msg.car_id.data = "";
                }
            }
        });

        // 添加碰撞体客户端
        add_collision_pub_ = this->create_publisher<fairino_msg::msg::CollisionObjectManage>("item/create_collision",10);

        // 传输二维码数据服务
        ItemMsg_service_ = this->create_service<fairino_msg::srv::ItemMsg>("item/itemMsg_trigger",std::bind(&PointCloudExtractor::onItemMsgTrigger, this,std::placeholders::_1, std::placeholders::_2));
    }

private:
    // 相机信息回调函数
    void depth_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // 仅在第一次接收到相机信息时更新参数
        if (!camera_info_received_)
        {
            fx = msg->k[0];
            fy = msg->k[4];
            cx = msg->k[2];
            cy = msg->k[5];
            RCLCPP_INFO(this->get_logger(), "相机内参已接收。");
            camera_info_received_ = true;
        }
    }

    // ros2服务Trigger回调函数
    void onItemMsgTrigger(
        const std::shared_ptr<fairino_msg::srv::ItemMsg::Request> request,
        const std::shared_ptr<fairino_msg::srv::ItemMsg::Response> response)
    {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        response->item_id = item_msg.item_id;
        response->car_id = item_msg.car_id;
    }

    // 同步回调函数
    void syncCallback(const fairino_msg::msg::QrMsg::ConstSharedPtr qr_msg,const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,const geometry_msgs::msg::PoseStamped::ConstSharedPtr aruco_pose)
    {
        // RCLCPP_INFO(this->get_logger(),"接收到二维码信息：item_id：%s，car_id：%s",qr_msg->item_id.data.c_str(),qr_msg->car_id.data.c_str());
        if (!camera_info_received_)
        {
            return;
        }

        // 获取目标点在图像中的像素坐标
        int u = static_cast<int>(qr_msg->center_point.x);
        int v = static_cast<int>(qr_msg->center_point.y);

        // 从深度图像中提取对应像素点的深度值
        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr depth_ptr;
        try
        {
            depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转化异常: %s", e.what());
            return;
        }

        cv::Mat depth_image = depth_ptr->image;
        if (v >= 0 && v < depth_image.rows && u >= 0 && u < depth_image.cols)
        {
            // 获取 world到camera_color_optical_frame的变换
            geometry_msgs::msg::TransformStamped transform;
            try
            {
                if (tf_buffer->canTransform("camera_color_optical_frame", "world", depth_msg->header.stamp, rclcpp::Duration::from_seconds(0.05))) {
                    transform = tf_buffer->lookupTransform("camera_color_optical_frame", "world", depth_msg->header.stamp);
                } else {
                    RCLCPP_WARN(this->get_logger(), "TF变换不可用，跳过本次处理");
                    return;
                }
            } catch (std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(),"获取 world到camera_color_optical_frame的变换 失败：%s",e.what());
                return;
            }

            // 从深度图像获取深度值
            uint16_t depth_value = depth_image.at<uint16_t>(v, u);

            if (depth_value>0)
            {
                // 使用相机内参计算相机坐标系下的 (x, y)
                double camera_z = static_cast<double>(depth_value) / 1000.0;   // 深度相机的深度值单位是mm，转换为m
                double camera_x = (u - cx) * camera_z / fx;
                double camera_y = (v - cy) * camera_z / fy;

                // 世界坐标系下的xyz
                double world_z = camera_z+transform.transform.translation.z;
                double world_x = camera_y+transform.transform.translation.y;
                double world_y = camera_x+transform.transform.translation.x;

                // 定义 CollisionObject，用于给机械臂抓取和放置
                if (!qr_msg->item_id.data.empty())
                {
                    fairino_msg::msg::CollisionObjectManage collision_object;
                    collision_object.id = qr_msg->item_id;
                    collision_object.manage_type.data = "add_item";
                    collision_object.pose_in_world.position.x = world_x;
                    collision_object.pose_in_world.position.y = world_y;
                    collision_object.pose_in_world.position.z = world_z;
                    collision_object.pose_in_world.orientation = aruco_pose->pose.orientation;
                    add_collision_pub_->publish(collision_object);
                }
                else if (!qr_msg->car_id.data.empty())
                {
                    fairino_msg::msg::CollisionObjectManage collision_object;
                    collision_object.id = qr_msg->car_id;
                    collision_object.manage_type.data = "add_car";
                    collision_object.pose_in_world.position.x = world_x;
                    collision_object.pose_in_world.position.y = world_y;
                    collision_object.pose_in_world.position.z = world_z;
                    collision_object.pose_in_world.orientation = aruco_pose->pose.orientation;
                    add_collision_pub_->publish(collision_object);
                }
                if (qr_msg->item_id!=item_msg.item_id || qr_msg->car_id!=item_msg.car_id)
                {
                    // 发布物品位置信息
                    std::lock_guard<std::mutex> lock(latest_mutex_);
                    item_msg.item_id = qr_msg->item_id;
                    item_msg.car_id = qr_msg->car_id;
                    RCLCPP_INFO(this->get_logger(), "更新 item_msg: item_id=%s, car_id=%s", item_msg.item_id.data.c_str(), item_msg.car_id.data.c_str());
                }
                qr_time_valid_ = true;
                last_qr_time_ = this->now();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),"深度值非法！");
                if (item_msg.item_id.data != "" || item_msg.car_id.data != "")
                {
                    std::lock_guard<std::mutex> lock(latest_mutex_);
                    item_msg.item_id.data = "";
                    item_msg.car_id.data = "";
                }
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000, "像素坐标（%d，%d）超出图像边界", u, v);
        }
    }

    // 成员变量
    bool qr_time_valid_ = false;
    rclcpp::Time last_qr_time_;
    std::mutex latest_mutex_;
    fairino_msg::srv::ItemMsg::Response item_msg;
    bool camera_info_received_ = false;
    double fx,fy,cx,cy;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Service<fairino_msg::srv::ItemMsg>::SharedPtr ItemMsg_service_;
    rclcpp::Publisher<fairino_msg::msg::CollisionObjectManage>::SharedPtr add_collision_pub_;
    rclcpp::TimerBase::SharedPtr qr_timeout_timer_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<fairino_msg::msg::QrMsg, sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>;
    std::shared_ptr<message_filters::Subscriber<fairino_msg::msg::QrMsg>> item_center_point_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> aruco_single_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudExtractor>());
    rclcpp::shutdown();
    return 0;
}