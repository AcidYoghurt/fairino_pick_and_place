#include <rclcpp/rclcpp.hpp>
#include <fairino_msg/msg/collision_object_manage.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class CollisionObjectManage : public rclcpp::Node
{
public:
    CollisionObjectManage() : Node("collision_object_manage")
    {
        // 参数
        this->declare_parameter("box_WDH", std::vector<double>{0.5, 0.2, 0.1});
        box_WDH = this->get_parameter("box_WDH").as_double_array();
        this->declare_parameter("car_WDH",std::vector<double>{0.5, 0.2, 0.1});
        car_WDH = this->get_parameter("car_WDH").as_double_array();
        collision_map = std::map<std::string,std::string>();

        // 创建碰撞体Sub
        create_collision_sub_ = this->create_subscription<fairino_msg::msg::CollisionObjectManage>("item/create_collision",10,
            std::bind(&CollisionObjectManage::add_collision_callback,this,std::placeholders::_1));

        // 删除碰撞体服务端
        delete_collision_service_ = this->create_service<std_srvs::srv::Trigger>("item/delete_collision_trigger",
            std::bind(&CollisionObjectManage::delete_collision_trigger,this,std::placeholders::_1,std::placeholders::_2));
    }

private:
    // 添加碰撞体
    void add_collision_callback(fairino_msg::msg::CollisionObjectManage collision_object_manage)
    {
        // 声明 CollisionObject
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";
        collision_object.id = collision_object_manage.id.data;

        // 定义几何体
        shape_msgs::msg::SolidPrimitive primitive;
        if (collision_object_manage.manage_type.data=="add_item")
        {
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = box_WDH[0];
            primitive.dimensions[primitive.BOX_Y] = box_WDH[1];
            primitive.dimensions[primitive.BOX_Z] = box_WDH[2];
        }
        else if (collision_object_manage.manage_type.data == "add_car")
        {
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = car_WDH[0];
            primitive.dimensions[primitive.BOX_Y] = car_WDH[1];
            primitive.dimensions[primitive.BOX_Z] = car_WDH[2];
        }

        // 定义位姿
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = collision_object_manage.pose_in_world.position.x;
        box_pose.position.y = collision_object_manage.pose_in_world.position.y;
        box_pose.position.z = collision_object_manage.pose_in_world.position.z-primitive.dimensions[primitive.BOX_Z]/2;
        box_pose.orientation = collision_object_manage.pose_in_world.orientation;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"添加【物品】碰撞体");
        if (collision_object.id!=last_collision_object_id)
        {
            collision_map.insert({collision_object.id.data(),collision_object.header.frame_id.data()});
            last_collision_object_id = collision_object.id;
        }

        planning_scene_interface.applyCollisionObject(collision_object);
    }

    // 删除碰撞体（Trigger）
    void delete_collision_trigger(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        for (const auto&[collision_id,collision_frame_id] : collision_map)
        {
            moveit_msgs::msg::CollisionObject obj;
            obj.id = collision_id;
            obj.header.frame_id = collision_frame_id;
            obj.operation = obj.REMOVE;

            moveit::planning_interface::PlanningSceneInterface psi;
            while (!psi.applyCollisionObject(obj)){}
            collision_map.erase(collision_id);
            RCLCPP_INFO(this->get_logger(),"已删除碰撞体：%s",collision_id.c_str());
        }

        // 回复
        response->success = true;
    }

    std::string last_collision_object_id;

    std::vector<double> box_WDH;
    std::vector<double> car_WDH;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::map<std::string,std::string> collision_map;

    rclcpp::Subscription<fairino_msg::msg::CollisionObjectManage>::SharedPtr create_collision_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr delete_collision_service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionObjectManage>());
    rclcpp::shutdown();
    return 0;
}