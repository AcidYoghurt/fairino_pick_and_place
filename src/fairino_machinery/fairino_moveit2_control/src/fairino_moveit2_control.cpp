#include <iostream>
#include <thread>
#include <string>
#include <cstring>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fairino_msg/srv/item_msg.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
namespace mtc = moveit::task_constructor;

//测试：
// ros2 topic pub /qr/item_position fairino_msg/msg/ItemPosition "{item_id: {data: \"item_001\"},cabinet_id: {data: \"cabinet_A\"},item_position: {header: {frame_id: \"base_link\"},point: {x: 0.5, y: 0.3, z: 0.2}}}"


class FairinoMoveit2Control : public rclcpp::Node
{
public:
    FairinoMoveit2Control(const rclcpp::NodeOptions& options) : Node("fairinoMoveit2ControlNode",options)
    {
        // 参数
        this->declare_parameter("robot_name","fairino5");
        robot_name = this->get_parameter("robot_name").as_string();

        // 变量
        arm_group_name = robot_name+"_v6_group";
        hand_group_name = "hand";
        hand_frame = "tool_frame";

        // subscriber和publisher
        ItemMsg_client_ = this->create_client<fairino_msg::srv::ItemMsg>("itemMsg_trigger");
        cabinet_points_sub_ = this->create_subscription<std_msgs::msg::String>("cabinet/points",10,std::bind(&FairinoMoveit2Control::cabinet_points_sub_callback,this,std::placeholders::_1));
        http_msg_sub_ = this->create_subscription<std_msgs::msg::String>("network/receiveMsg",10,std::bind(&FairinoMoveit2Control::fairino_move,this,std::placeholders::_1));
        http_msg_pub_ = this->create_publisher<std_msgs::msg::String>("network/sendMsg",10);
    }

private:
    // 获取柜子的点位
    void cabinet_points_sub_callback(std_msgs::msg::String msg)
    {
        if (!is_cabinet_points_received)
        {
            try
            {
                nlohmann::json j = nlohmann::json::parse(msg.data);
                for (auto& [key_str, point_array] : j.items())
                {
                    int x,y;
                    if (sscanf(key_str.c_str(), "(%d, %d)", &x, &y) == 2) {
                        cabinet_points_map.insert({{x,y},{point_array[0],point_array[1],point_array[2]}});
                        is_cabinet_points_received = true;
                    }else
                    {
                        RCLCPP_ERROR(this->get_logger(),"cabinet_points数据解析错误！");
                        cabinet_points_map.clear();
                        return;
                    }
                }
                RCLCPP_INFO(this->get_logger(),"cabinet_points数据已接收！");
            } catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(),"无法解析 cabinet_points json数据：%s",e.what());
            }
        }
    }

    // 执行机械臂序列
    void fairino_move(std_msgs::msg::String msg)
    {
        // 查看柜子点位是否被接收
        if (!is_cabinet_points_received)
        {
            RCLCPP_WARN(this->get_logger(),"%s","柜子点位未被接收，机械臂仍在初始化！");
            return;
        }

        /* 接收网络信息 */
        std::string message_type;
        std::string car_id;
        std::string item_id;
        std::pair<int,int> cabinet_id = std::pair<int,int>();
        // 解析json数据
        try
        {
            nlohmann::json j = nlohmann::json::parse(msg.data);
            if (j.contains("message_type"))
            {
                if (j["message_type"]=="item_store")
                {
                    message_type = j["message_type"];
                    cabinet_id.first = j["cabinet_id"][0];
                    cabinet_id.second = j["cabinet_id"][1];
                }
                else if (j["message_type"]=="item_outbound")
                {
                    message_type = j["message_type"];
                    car_id = j["car_id"];
                    item_id = j["item_id"];
                    cabinet_id.first = j["cabinet_id"][0];
                    cabinet_id.second = j["cabinet_id"][1];
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),"message_type信息错误！");
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", "receive_netMsg"},
                        {"task_status", "fail"},
                        {"task_msg", "message_type信息错误"}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }
            }
            else
                return;
        } catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"无法解析 网络消息 json数据：%s",e.what());
            std_msgs::msg::String str;
            str.data = (nlohmann::json{
                {"message_type", "receive_netMsg"},
                {"task_status", "fail"},
                {"task_msg", "无法解析 网络消息 json数据: "+std::string(e.what())}
            }).dump();
            http_msg_pub_->publish(str);
            return;
        }
        // 检查数据是否合法
        std::vector<double> cabinet_point = std::vector<double>(3);
        auto it = cabinet_points_map.find(cabinet_id);
        if (it != cabinet_points_map.end())
        {
            cabinet_point = it->second;
        }

        // 初始化机械臂
        moveit::planning_interface::MoveGroupInterface move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), arm_group_name); // 移动组接口
        move_group.setNamedTarget("pos1");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "初始化机械臂【规划】失败！");
            return;
        }
        if (!move_group.move()==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "初始化机械臂【运行】失败！");
            return;
        }
        // 打开夹爪
        moveit::planning_interface::MoveGroupInterface hand_group(this->shared_from_this(), hand_group_name);
        hand_group.setNamedTarget("open");
        if (!hand_group.move()==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "夹爪打开失败");
            return;
        }

        if (message_type=="item_store")
        {
            // 根据CollisionObject的id来获取该物品
            moveit::planning_interface::PlanningSceneInterface psi;
            moveit_msgs::msg::CollisionObject obj;
            geometry_msgs::msg::Pose object_pose;
            while (rclcpp::ok())
            {
                std::map<std::string, moveit_msgs::msg::CollisionObject> objects = psi.getObjects({item_id});
                if (!objects.empty() && objects.find(item_id) != objects.end())
                {
                    // 正确获取物体对象
                    moveit_msgs::msg::CollisionObject obj = objects[item_id];

                    // 检查是否有primitive_poses
                    if (!obj.primitive_poses.empty())
                    {
                        object_pose = obj.primitive_poses[0];
                        RCLCPP_INFO(this->get_logger(), "成功获取物品%s位姿",item_id.data());
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            {   // task1
                auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());  // 调用 MoveIt 的标准 OMPL 采样规划器
                auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();                // 关节插值规划器，用于张开/闭合夹爪
                auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                                // 笛卡尔路径规划器，用于末端移动

                auto task = std::make_unique<mtc::Task>();
                task->loadRobotModel(this->shared_from_this());
                task->stages()->setName("lift_item");
                task->setProperty("group",arm_group_name);
                task->setProperty("eef",hand_group_name);
                task->setProperty("ik_frame",hand_frame);

                // 添加stage
                {   // 初始状态
                    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
                    task->add(std::move(current_state));
                }
                {   // 允许夹爪与物品碰撞
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
                    // 获取机械手所有具有碰撞几何的链接名称
                    auto hand_links = task->getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry();
                    stage->allowCollisions(obj.id,hand_links,true);
                    task->add(std::move(stage));
                }
                {   // 移动到物品
                    auto move_to_item_stage = std::make_unique<mtc::stages::MoveTo>("approach item",sampling_planner);
                    move_to_item_stage->setGroup(arm_group_name);
                    move_to_item_stage->setIKFrame(hand_frame);

                    geometry_msgs::msg::PoseStamped target_pose;
                    target_pose.header.frame_id = "world";
                    target_pose.pose.position.x = object_pose.position.x;
                    target_pose.pose.position.y = object_pose.position.y;
                    target_pose.pose.position.z = object_pose.position.z;

                    tf2::Quaternion obj_orien,temp,target_orien;
                    tf2::fromMsg(object_pose.orientation,obj_orien);
                    temp.setRPY(0,M_PI,0);
                    target_orien = obj_orien * temp;
                    target_orien.normalize();
                    target_pose.pose.orientation = tf2::toMsg(target_orien);

                    move_to_item_stage->setGoal(target_pose);
                    task->add(std::move(move_to_item_stage));
                }
                {   // 关闭夹爪
                    auto close_gripper_stage =  std::make_unique<mtc::stages::MoveTo>("close gripper",interpolation_planner);
                    close_gripper_stage->setGroup(hand_group_name);
                    close_gripper_stage->setGoal("close");
                    task->add(std::move(close_gripper_stage));
                }
                {   // 将物体附加到末端执行器
                    auto attach_item_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
                    attach_item_stage->attachObject(obj.id,hand_frame);
                    task->add(std::move(attach_item_stage));
                }

                /* 初始化任务 */
                try
                {
                    task->init();
                } catch (mtc::InitStageException& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "初始化任务失败：%s",e.what());
                    return;
                }

                /* 任务规划 */
                if (!task->plan(100))
                {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 规划失败！",task->name().c_str());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "任务规划失败"}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }

                /* 任务执行 */
                task->introspection().publishSolution(*task->solutions().front());
                auto task_result = task->execute(*task->solutions().front());
                if (task_result.val != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 执行失败！", task->name().c_str());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "任务执行失败"}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }
            }

            {   // task2
                auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());  // 调用 MoveIt 的标准 OMPL 采样规划器
                auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();                // 关节插值规划器，用于张开/闭合夹爪
                auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                                // 笛卡尔路径规划器，用于末端移动


                auto task = std::make_unique<mtc::Task>();
                task->loadRobotModel(this->shared_from_this());
                task->stages()->setName("putdown_item");
                task->setProperty("group",arm_group_name);
                task->setProperty("eef",hand_group_name);
                task->setProperty("ik_frame",hand_frame);

                // 添加stage
                {   // 初始状态
                    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
                    task->add(std::move(current_state));
                }
                {   // 抬起物体
                    auto lift_item_stage = std::make_unique<mtc::stages::MoveRelative>("lift object",cartesian_planner);
                    lift_item_stage->setGroup(arm_group_name);
                    lift_item_stage->setIKFrame(hand_frame);

                    geometry_msgs::msg::Vector3Stamped lift_position;
                    lift_position.header.frame_id = "world";
                    lift_position.vector.z = 0.1;
                    lift_item_stage->setDirection(lift_position);
                    task->add(std::move(lift_item_stage));
                }
                {   // 放置物体
                    auto move_to_item_stage = std::make_unique<mtc::stages::MoveTo>("place item",sampling_planner);
                    move_to_item_stage->setGroup(arm_group_name);
                    move_to_item_stage->setIKFrame(hand_frame);

                    geometry_msgs::msg::PoseStamped target_pose;
                    target_pose.header.frame_id = "world";
                    target_pose.pose.position.x = cabinet_point[0];
                    target_pose.pose.position.y = cabinet_point[1];
                    target_pose.pose.position.z = cabinet_point[2]+0.05;

                    tf2::Quaternion temp;
                    temp.setRPY(-M_PI/2, 0, -M_PI/2);
                    target_pose.pose.orientation = tf2::toMsg(temp);

                    move_to_item_stage->setGoal(target_pose);
                    task->add(std::move(move_to_item_stage));
                }
                {   // 打开夹爪
                    auto open_gripper_stage =  std::make_unique<mtc::stages::MoveTo>("open gripper again",interpolation_planner);
                    open_gripper_stage->setGroup(hand_group_name);
                    open_gripper_stage->setGoal("open");
                    task->add(std::move(open_gripper_stage));
                }
                {   // 分离对象
                    auto detachObject_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
                    detachObject_stage->detachObject(obj.id, hand_frame);
                    task->add(std::move(detachObject_stage));
                }
                {   // 禁止夹爪与物品碰撞
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
                    stage->allowCollisions(obj.id,task->getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),false);
                    task->add(std::move(stage));
                }
                {   // 夹爪后退
                    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
                    stage->setGroup(arm_group_name);
                    stage->setIKFrame(hand_frame);
                    stage->setMinMaxDistance(0.05, 0.30);
                    // 设置撤退方向
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = hand_frame;
                    vec.vector.z = -0.1;
                    stage->setDirection(vec);
                    task->add(std::move(stage));
                }
                {   // 返回原点
                    auto return_origin_stage = std::make_unique<mtc::stages::MoveTo>("return origin",sampling_planner);
                    return_origin_stage->setGroup(arm_group_name);
                    return_origin_stage->setGoal("pos1");
                    task->add(std::move(return_origin_stage));
                }

                /* 初始化任务 */
                try
                {
                    task->init();
                } catch (mtc::InitStageException& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "初始化任务失败：%s",e.what());
                    return;
                }

                /* 任务规划 */
                if (!task->plan(100))
                {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 规划失败！",task->name().c_str());
                    return;
                }

                /* 任务执行 */
                task->introspection().publishSolution(*task->solutions().front());
                auto task_result = task->execute(*task->solutions().front());
                if (task_result.val != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 执行失败！", task->name().c_str());
                    return;
                }

                std_msgs::msg::String str;
                str.data = (nlohmann::json{
                    {"message_type", task->name()},
                    {"task_status", "success"},
                    {"item_id", obj.id},
                    {"cabinet_id", {cabinet_id.first, cabinet_id.second}}
                }).dump();
                http_msg_pub_->publish(str);
            }
        }
        else if (message_type=="item_outbound")
        {
            // TODO:监测小车是否到达
            // while (true)
            // {
            //     auto srv_request = std::make_shared<fairino_msg::srv::ItemMsg::Request>();
            //     auto srv_result = ItemMsg_client_->async_send_request(srv_request);
            //
            //     auto future_status = srv_result.wait_for(std::chrono::seconds(4)); //超时等待4s
            //     if (future_status == std::future_status::ready) {
            //         // 服务调用完成
            //         try {
            //             if (srv_result.get()->car_id.data == car_id)
            //                 break;
            //         } catch (const std::exception& e) {
            //             RCLCPP_ERROR(this->get_logger(), "服务响应异常: %s", e.what());
            //         }
            //     } else if (future_status == std::future_status::timeout) {
            //         ItemMsg_client_->remove_pending_request(srv_result);
            //     } else {
            //         RCLCPP_ERROR(this->get_logger(), "服务调用被延迟");
            //     }
            //     sleep(1);
            // }
            moveit::planning_interface::PlanningSceneInterface psi;

            {   // task1
                auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());     // 调用 MoveIt 的标准 OMPL 采样规划器
                auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();                   // 关节插值规划器，用于张开/闭合夹爪
                auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                                   // 笛卡尔路径规划器，用于末端移动

                auto task = std::make_unique<mtc::Task>();
                task->loadRobotModel(this->shared_from_this());
                task->stages()->setName("item_store");
                task->setProperty("group",arm_group_name);
                task->setProperty("eef",hand_group_name);
                task->setProperty("ik_frame",hand_frame);

                // 添加stage
                {   // 初始状态
                    auto current_stage = std::make_unique<mtc::stages::CurrentState>("current state");
                    task->add(std::move(current_stage));
                }
                {   // 前往柜子
                    auto stage = std::make_unique<mtc::stages::MoveTo>("move to cabinet",sampling_planner);
                    stage->setGroup(arm_group_name);
                    stage->setIKFrame(hand_frame);

                    geometry_msgs::msg::PoseStamped target_pose;
                    target_pose.header.frame_id = "world";
                    target_pose.pose.position.x = cabinet_point[0]-0.05;
                    target_pose.pose.position.y = cabinet_point[1];
                    target_pose.pose.position.z = cabinet_point[2]+0.05;

                    tf2::Quaternion temp;
                    temp.setRPY(-M_PI/2, 0, -M_PI/2);
                    target_pose.pose.orientation = tf2::toMsg(temp);

                    stage->setGoal(target_pose);
                    task->add(std::move(stage));
                }

                /* 初始化任务 */
                try
                {
                    task->init();
                } catch (mtc::InitStageException& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "初始化任务失败：%s",e.what());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "初始化任务失败: "+std::string(e.what())}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }

                /* 任务规划 */
                if (!task->plan(100))
                {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 规划失败！",task->name().c_str());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "任务规划失败"}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }

                /* 任务执行 */
                task->introspection().publishSolution(*task->solutions().front());
                auto task_result = task->execute(*task->solutions().front());
                if (task_result.val != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 执行失败！", task->name().c_str());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "任务执行失败"}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }
            }

            // 根据CollisionObject的id来获取物品
            geometry_msgs::msg::Pose object_pose;
            moveit_msgs::msg::CollisionObject obj;
            while (rclcpp::ok())
            {
                std::map<std::string, moveit_msgs::msg::CollisionObject> objects = psi.getObjects({item_id});

                if (!objects.empty() && objects.find(item_id) != objects.end())
                {
                    // 正确获取物体对象
                    obj = objects[item_id];
                    object_pose = obj.pose;

                    // 这里可以安全使用object_pose
                    RCLCPP_INFO(this->get_logger(), "成功获取物品位姿");
                    break;
                }
                RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),5000,"暂未接收到要抓取的 item 位姿，任务中断！");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            {   // task2
                auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());     // 调用 MoveIt 的标准 OMPL 采样规划器
                auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();                   // 关节插值规划器，用于张开/闭合夹爪
                auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                                   // 笛卡尔路径规划器，用于末端移动

                auto task = std::make_unique<mtc::Task>();
                task->loadRobotModel(this->shared_from_this());
                task->stages()->setName("item_store");
                task->setProperty("group",arm_group_name);
                task->setProperty("eef",hand_group_name);
                task->setProperty("ik_frame",hand_frame);

                // 添加stage
                {   // 初始状态
                    auto current_stage = std::make_unique<mtc::stages::CurrentState>("current state");
                    task->add(std::move(current_stage));
                }
                {   // 允许夹爪与物品碰撞
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
                    // 获取机械手所有具有碰撞几何的链接名称
                    auto hand_links = task->getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry();
                    stage->allowCollisions(obj.id,hand_links,true);
                    task->add(std::move(stage));
                }
                {   // 靠近物品
                    auto stage = std::make_unique<mtc::stages::MoveTo>("approach item",sampling_planner);
                    stage->setGroup(arm_group_name);
                    stage->setIKFrame(hand_frame);

                    geometry_msgs::msg::PoseStamped target_pose;
                    target_pose.header.frame_id = "world";
                    target_pose.pose.position.x = object_pose.position.x;
                    target_pose.pose.position.y = object_pose.position.y;
                    target_pose.pose.position.z = object_pose.position.z;

                    tf2::Quaternion temp;
                    temp.setRPY(-M_PI/2, 0, -M_PI/2);
                    target_pose.pose.orientation = tf2::toMsg(temp);

                    stage->setGoal(target_pose);
                    task->add(std::move(stage));
                }
                {   // 关闭夹爪
                    auto close_gripper_stage =  std::make_unique<mtc::stages::MoveTo>("close gripper",interpolation_planner);
                    close_gripper_stage->setGroup(hand_group_name);
                    close_gripper_stage->setGoal("close");
                    task->add(std::move(close_gripper_stage));
                }
                {   // 将物体附加到末端执行器
                    auto attach_item_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
                    attach_item_stage->attachObject(obj.id,hand_frame);
                    task->add(std::move(attach_item_stage));
                }

                /* 初始化任务 */
                try
                {
                    task->init();
                } catch (mtc::InitStageException& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "初始化任务失败：%s",e.what());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "初始化任务失败: "+std::string(e.what())}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }

                /* 任务规划 */
                if (!task->plan(100))
                {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 规划失败！",task->name().c_str());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "任务规划失败"}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }

                /* 任务执行 */
                task->introspection().publishSolution(*task->solutions().front());
                auto task_result = task->execute(*task->solutions().front());
                if (task_result.val != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 执行失败！", task->name().c_str());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "任务执行失败"}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }
            }

            {   // task3
                auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());     // 调用 MoveIt 的标准 OMPL 采样规划器
                auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();                   // 关节插值规划器，用于张开/闭合夹爪
                auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                                   // 笛卡尔路径规划器，用于末端移动

                auto task = std::make_unique<mtc::Task>();
                task->loadRobotModel(this->shared_from_this());
                task->stages()->setName("item_store");
                task->setProperty("group",arm_group_name);
                task->setProperty("eef",hand_group_name);
                task->setProperty("ik_frame",hand_frame);

                {   // 初始状态
                    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
                    task->add(std::move(current_state));
                }
                {   // 抬起物体
                    auto lift_item_stage = std::make_unique<mtc::stages::MoveRelative>("lift object",cartesian_planner);
                    lift_item_stage->setGroup(arm_group_name);
                    lift_item_stage->setIKFrame(hand_frame);
                    lift_item_stage->setMinMaxDistance(0.05, (cabinet_point[2]-object_pose.position.z)/2);

                    geometry_msgs::msg::Vector3Stamped lift_position;
                    lift_position.header.frame_id = "world";
                    lift_position.vector.z = 1;
                    lift_item_stage->setDirection(lift_position);
                    task->add(std::move(lift_item_stage));
                }
                {   // 物体向后移动（出柜）
                    auto item_backward_stage = std::make_unique<mtc::stages::MoveRelative>("object backward",cartesian_planner);
                    item_backward_stage->setGroup(arm_group_name);
                    item_backward_stage->setIKFrame(hand_frame);
                    item_backward_stage->setMinMaxDistance(cabinet_point[1], cabinet_point[1]+0.30);

                    geometry_msgs::msg::Vector3Stamped lift_position;
                    lift_position.header.frame_id = "world";
                    lift_position.vector.x = -1;
                    item_backward_stage->setDirection(lift_position);
                    task->add(std::move(item_backward_stage));
                }
                {   // 到小车上方（原点）
                    auto return_origin_stage = std::make_unique<mtc::stages::MoveTo>("return origin",sampling_planner);
                    return_origin_stage->setGroup(arm_group_name);
                    return_origin_stage->setGoal("pos1");
                    task->add(std::move(return_origin_stage));
                }
                {   // 向下降
                    auto move_down_stage = std::make_unique<mtc::stages::MoveRelative>("move down",cartesian_planner);
                    move_down_stage->setGroup(arm_group_name);
                    move_down_stage->setIKFrame(hand_frame);

                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = "world";
                    vec.vector.z = -0.20;
                    move_down_stage->setDirection(vec);
                    task->add(std::move(move_down_stage));
                }
                {   // 打开夹爪
                    auto open_gripper_stage =  std::make_unique<mtc::stages::MoveTo>("open gripper again",interpolation_planner);
                    open_gripper_stage->setGroup(hand_group_name);
                    open_gripper_stage->setGoal("open");
                    task->add(std::move(open_gripper_stage));
                }
                {   // 分离对象
                    auto detachObject_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
                    detachObject_stage->detachObject(obj.id, hand_frame);
                    task->add(std::move(detachObject_stage));
                }
                {   // 禁止夹爪与物品碰撞
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
                    stage->allowCollisions(obj.id,task->getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),false);
                    task->add(std::move(stage));
                }
                {   // 夹爪后退
                    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
                    stage->setGroup(arm_group_name);
                    stage->setIKFrame(hand_frame);
                    stage->setMinMaxDistance(0.05, 0.30);
                    // 设置撤退方向
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = hand_frame;
                    vec.vector.z = 1;
                    stage->setDirection(vec);
                    task->add(std::move(stage));
                }
                {   // 返回原点
                    auto return_origin_stage = std::make_unique<mtc::stages::MoveTo>("return origin",sampling_planner);
                    return_origin_stage->setGroup(arm_group_name);
                    return_origin_stage->setGoal("pos1");
                    task->add(std::move(return_origin_stage));
                }

                /* 初始化任务 */
                try
                {
                    task->init();
                } catch (mtc::InitStageException& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "初始化任务失败：%s",e.what());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "初始化任务失败: "+std::string(e.what())}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }

                /* 任务规划 */
                if (!task->plan(100))
                {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 规划失败！",task->name().c_str());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "任务规划失败"}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }

                /* 任务执行 */
                task->introspection().publishSolution(*task->solutions().front());
                auto task_result = task->execute(*task->solutions().front());
                if (task_result.val != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "任务：%s 执行失败！", task->name().c_str());
                    std_msgs::msg::String str;
                    str.data = (nlohmann::json{
                        {"message_type", task->name()},
                        {"task_status", "fail"},
                        {"task_msg", "任务执行失败"}
                    }).dump();
                    http_msg_pub_->publish(str);
                    return;
                }
            }
        }
    }

    // 机械臂参数
    std::string robot_name;
    std::string arm_group_name;
    std::string hand_group_name;
    std::string hand_frame;
    std::map<std::pair<int,int>,std::vector<double>> cabinet_points_map;

    bool is_cabinet_points_received = false;

    // Pub和Sub
    rclcpp::Client<fairino_msg::srv::ItemMsg>::SharedPtr ItemMsg_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr http_msg_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr http_msg_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cabinet_points_sub_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<FairinoMoveit2Control>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
