#include <rclcpp/rclcpp.hpp>
#include <fairino_msg/msg/qr_msg.hpp>
#include <fairino_msg/srv/item_msg.hpp>
#include <fairino_msg/msg/collision_object_manage.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // 新增，用于订阅世界坐标系下的位姿
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class CollisionManagerNode : public rclcpp::Node
{
public:
    // 节点名称修改以反映其新职责
    CollisionManagerNode() : Node("collision_manager_node")
    {
        // 创建消息过滤器订阅器，订阅二维码 ID/像素信息 和 世界坐标系下的位姿
        qr_msg_sub_ = std::make_shared<message_filters::Subscriber<fairino_msg::msg::QrMsg>>(this, "qr/qr_msg");
        // 订阅由 QrPoseEstimatorNode 发布的 3D 位姿
        pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, "qr/item_pose_world");

        // 使用近似时间同步策略来同步 QrMsg 和其对应的 3D 位姿
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), 
            *qr_msg_sub_, 
            *pose_sub_
        );
        // 回调函数只接收两个参数：qr_msg 和 pose_stamped
        sync_->registerCallback(std::bind(&CollisionManagerNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        // 二维码超时清空计时器 (保留)
        qr_timeout_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),[this]()
        {
            if (qr_time_valid_==true)
            {
                // 超时判断逻辑保留
                if (((this->now() - last_qr_time_).seconds() > 0.5) && (item_msg.item_id.data != "" || item_msg.car_id.data != ""))
                {
                    RCLCPP_INFO(this->get_logger(),"二维码超时，清空 item_msg");
                    std::lock_guard<std::mutex> lock(latest_mutex_);
                    item_msg.item_id.data = "";
                    item_msg.car_id.data = "";
                }
            }
        });

        // 添加碰撞体发布器 (保留)
        add_collision_pub_ = this->create_publisher<fairino_msg::msg::CollisionObjectManage>("item/create_collision",10);

        // 传输二维码数据服务 (保留)
        ItemMsg_service_ = this->create_service<fairino_msg::srv::ItemMsg>("item/itemMsg_trigger",std::bind(&CollisionManagerNode::onItemMsgTrigger, this,std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "CollisionManagerNode (原 PointCloudExtractor) 初始化完成。");
    }

private:
    // 移除了 depth_camera_info_callback
    // 移除了 fx, fy, cx, cy 和 camera_info_received_ 变量

    // ros2服务Trigger回调函数 (保留)
    void onItemMsgTrigger(
        const std::shared_ptr<fairino_msg::srv::ItemMsg::Request> request,
        const std::shared_ptr<fairino_msg::srv::ItemMsg::Response> response)
    {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        response->item_id = item_msg.item_id;
        response->car_id = item_msg.car_id;
    }

    // 同步回调函数：现在只接收 QrMsg 和 3D PoseStamped
    void syncCallback(
        const fairino_msg::msg::QrMsg::ConstSharedPtr qr_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_stamped_msg) // 新增 PoseStamped 消息
    {


        // 从 PoseStamped 消息中直接获取世界坐标系下的位姿
        const geometry_msgs::msg::Pose& pose_in_world = pose_stamped_msg->pose;
        
        // 检查 PoseStamped 的 frame_id 是否是预期的世界坐标系
        if (pose_stamped_msg->header.frame_id != "world")
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "接收到的位姿不是在世界坐标系 ('%s') 下，跳过碰撞体发布。", pose_stamped_msg->header.frame_id.c_str());
            return;
        }

        // 定义 CollisionObject，用于给机械臂抓取和放置
        if (!qr_msg->item_id.data.empty())
        {
            fairino_msg::msg::CollisionObjectManage collision_object;
            collision_object.id = qr_msg->item_id;
            collision_object.manage_type.data = "add_item";
            
            // 直接使用 PoseStamped 的位置和方向
            collision_object.pose_in_world.position = pose_in_world.position;
            collision_object.pose_in_world.orientation = pose_in_world.orientation;
            
            add_collision_pub_->publish(collision_object);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "发布货物碰撞体: ID=%s", qr_msg->item_id.data.c_str());
        }
        else if (!qr_msg->car_id.data.empty())
        {
            fairino_msg::msg::CollisionObjectManage collision_object;
            collision_object.id = qr_msg->car_id;
            collision_object.manage_type.data = "add_car";
            
            // 直接使用 PoseStamped 的位置和方向
            collision_object.pose_in_world.position = pose_in_world.position;
            collision_object.pose_in_world.orientation = pose_in_world.orientation;
            
            add_collision_pub_->publish(collision_object);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "发布小车碰撞体: ID=%s", qr_msg->car_id.data.c_str());
        }
        else
        {
            // 如果 qr_msg 既没有 item_id 也没有 car_id，则不发布碰撞体
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "QrMsg 不包含有效的 item_id 或 car_id。");
            return;
        }

        // 更新最新信息 
        if (qr_msg->item_id.data != item_msg.item_id.data || qr_msg->car_id.data != item_msg.car_id.data)
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

    // 成员变量
    bool qr_time_valid_ = false;
    rclcpp::Time last_qr_time_;
    std::mutex latest_mutex_;
    fairino_msg::srv::ItemMsg::Response item_msg;



    rclcpp::Service<fairino_msg::srv::ItemMsg>::SharedPtr ItemMsg_service_;
    rclcpp::Publisher<fairino_msg::msg::CollisionObjectManage>::SharedPtr add_collision_pub_;
    rclcpp::TimerBase::SharedPtr qr_timeout_timer_;

    // 同步策略：现在只同步 QrMsg 和 PoseStamped
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<fairino_msg::msg::QrMsg, geometry_msgs::msg::PoseStamped>;
    std::shared_ptr<message_filters::Subscriber<fairino_msg::msg::QrMsg>> qr_msg_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_sub_; // 新的订阅者
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionManagerNode>()); // 更改了节点名称
    rclcpp::shutdown();
    return 0;
}