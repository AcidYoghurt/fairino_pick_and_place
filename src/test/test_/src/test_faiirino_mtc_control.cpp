#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <nlohmann/json.hpp>
namespace mtc = moveit::task_constructor;

class TestFaitinoMoveit2Control : public rclcpp::Node
{
public:
    TestFaitinoMoveit2Control(const rclcpp::NodeOptions& options):Node("TestFaitinoMoveit2ControlNode",options)
    {
        // 变量
        move_group_interface_ = nullptr;
        robot_name = "fairino5";
        arm_group_name = robot_name+"_v6_group";
        hand_group_name = "hand";
        hand_frame = "tool_frame";

        tf_buffer  = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        cabinet_points_sub_ = this->create_subscription<std_msgs::msg::String>("cabinet/points",10,std::bind(&TestFaitinoMoveit2Control::cabinet_points_sub_callback,this,std::placeholders::_1));
    }

    void add_collision()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),arm_group_name);

        {
            // 创建碰撞物体
            moveit_msgs::msg::CollisionObject collision_object;

            collision_object.id = "box1";
            collision_object.header.frame_id = "world";

            // 定义形状
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            primitive.dimensions = {0.05, 0.05, 0.05};
            collision_object.primitives.push_back(primitive);


            // 定义位姿
            geometry_msgs::msg::Pose box_pose;
            box_pose.position.x = -0.4;
            box_pose.position.y = -0.4;
            box_pose.position.z = 0.0;
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, M_PI / 4);
            box_pose.orientation = tf2::toMsg(orientation);
            collision_object.pose = box_pose;

            geometry_msgs::msg::Pose primitive_pose;
            primitive_pose.orientation.w = 1.0;
            collision_object.primitive_poses.push_back(primitive_pose);

            // 添加到场景
            collision_object.operation = collision_object.ADD;
            psi.applyCollisionObject(collision_object);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        {
            // 创建碰撞物体
            moveit_msgs::msg::CollisionObject collision_object;

            collision_object.id = "aaaaaaaaa";
            collision_object.header.frame_id = "world";

            // 定义形状
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            primitive.dimensions = {0.10, 0.10, 0.02};
            collision_object.primitives.push_back(primitive);


            // 定义位姿
            geometry_msgs::msg::Pose box_pose;
            box_pose.position.x = -0.4;
            box_pose.position.y = -0.4;
            box_pose.position.z = -0.04;
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, 0);
            box_pose.orientation = tf2::toMsg(orientation);
            collision_object.pose = box_pose;

            geometry_msgs::msg::Pose primitive_pose;
            primitive_pose.orientation.w = 1.0;
            collision_object.primitive_poses.push_back(primitive_pose);

            // 添加到场景
            collision_object.operation = collision_object.ADD;
            psi.applyCollisionObject(collision_object);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        {
            // 创建碰撞物体
            moveit_msgs::msg::CollisionObject collision_object;

            collision_object.id = "cabinet";
            collision_object.header.frame_id = "world";

            // 定义形状
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            primitive.dimensions = {0.10, 0.10, 0.04};
            collision_object.primitives.push_back(primitive);


            // 定义位姿
            geometry_msgs::msg::Pose box_pose;
            box_pose.position.x = 0.4;
            box_pose.position.y = 0.4;
            box_pose.position.z = 0.4;
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, 0);
            box_pose.orientation = tf2::toMsg(orientation);
            collision_object.pose = box_pose;

            geometry_msgs::msg::Pose primitive_pose;
            primitive_pose.orientation.w = 1.0;
            collision_object.primitive_poses.push_back(primitive_pose);

            // 添加到场景
            collision_object.operation = collision_object.ADD;
            psi.applyCollisionObject(collision_object);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    void machinery_move()
    {
        if (is_cabinet_points_received)
        {
            // 检查数据是否合法
            std::pair<int,int> cabinet_id = std::pair<int,int>(0,0);
            std::vector<double> cabinet_point = std::vector<double>(3);
            auto it = cabinet_points_map.find(cabinet_id);
            if (it != cabinet_points_map.end())
            {
                cabinet_point = it->second;
            }


            {
                // 根据CollisionObject的id来获取该物品
                geometry_msgs::msg::Pose object_pose;
                moveit_msgs::msg::CollisionObject obj;
                while (rclcpp::ok())
                {
                    std::map<std::string, moveit_msgs::msg::CollisionObject> objects = psi.getObjects({"box1"});

                    if (!objects.empty() && objects.find("box1") != objects.end())
                    {
                        // 正确获取物体对象
                        obj = objects["box1"];

                        object_pose = obj.pose;

                        // 这里可以安全使用object_pose
                        RCLCPP_INFO(this->get_logger(), "成功获取物品位姿");
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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

                {   // task1
                    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());  // 调用 MoveIt 的标准 OMPL 采样规划器
                    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();                // 关节插值规划器，用于张开/闭合夹爪
                    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                                // 笛卡尔路径规划器，用于末端移动

                    auto task = std::make_unique<mtc::Task>();
                    task->loadRobotModel(this->shared_from_this());
                    task->stages()->setName("item_store");
                    task->setProperty("group",arm_group_name);
                    task->setProperty("eef",hand_group_name);
                    task->setProperty("ik_frame",hand_frame);

                    // 添加stage
                    mtc::Stage* current_state_ptr = nullptr;
                    {   // 初始状态
                        auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
                        current_state_ptr = current_state.get();
                        task->add(std::move(current_state));
                    }
                    {   // 允许夹爪与物品碰撞
                        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
                        // 获取机械手所有具有碰撞几何的链接名称
                        auto hand_links = task->getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry();
                        stage->allowCollisions(obj.id,hand_links,true);
                        task->add(std::move(stage));
                    }
                    {   // 移动到物品上方
                        auto move_to_item_stage = std::make_unique<mtc::stages::MoveTo>("approach item",sampling_planner);
                        move_to_item_stage->setGroup(arm_group_name);
                        move_to_item_stage->setIKFrame(hand_frame);

                        geometry_msgs::msg::PoseStamped target_pose;
                        target_pose.header.frame_id = "world";
                        target_pose.pose.position.x = object_pose.position.x;
                        target_pose.pose.position.y = object_pose.position.y;
                        target_pose.pose.position.z = object_pose.position.z+0.1;

                        tf2::Quaternion obj_orien,temp,target_orien;
                        tf2::fromMsg(object_pose.orientation,obj_orien);
                        temp.setRPY(0,M_PI,0);
                        target_orien = obj_orien * temp;
                        target_orien.normalize();
                        target_pose.pose.orientation = tf2::toMsg(target_orien);

                        move_to_item_stage->setGoal(target_pose);
                        task->add(std::move(move_to_item_stage));
                    }
                    {   // 移动到物品
                        auto stage = std::make_unique<mtc::stages::MoveRelative>("move down",cartesian_planner);
                        stage->setGroup(arm_group_name);
                        stage->setIKFrame(hand_frame);

                        geometry_msgs::msg::Vector3Stamped vector;
                        vector.header.frame_id = "world";
                        vector.vector.z = -0.1;
                        stage->setDirection(vector);
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
                }

                {   // task2
                    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());  // 调用 MoveIt 的标准 OMPL 采样规划器
                    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();                // 关节插值规划器，用于张开/闭合夹爪
                    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                                // 笛卡尔路径规划器，用于末端移动

                    auto task = std::make_unique<mtc::Task>();
                    task->loadRobotModel(this->shared_from_this());
                    task->stages()->setName("item_store");
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
                        target_pose.pose.position.x = cabinet_point[0]-0.1;
                        target_pose.pose.position.y = cabinet_point[1];
                        target_pose.pose.position.z = cabinet_point[2]+0.05;

                        tf2::Quaternion temp;
                        temp.setRPY(-M_PI/2, 0, -M_PI/2);
                        target_pose.pose.orientation = tf2::toMsg(temp);

                        move_to_item_stage->setGoal(target_pose);
                        task->add(std::move(move_to_item_stage));
                    }
                    {   // 向前移动
                        auto stage = std::make_unique<mtc::stages::MoveRelative>("move forward",cartesian_planner);
                        stage->setGroup(arm_group_name);
                        stage->setIKFrame(hand_frame);

                        geometry_msgs::msg::Vector3Stamped vector;
                        vector.header.frame_id = "world";
                        vector.vector.x = 0.1;
                        stage->setDirection(vector);
                        task->add(std::move(stage));
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
                }
            }

//////////////////////////////////////////////////////////////////////// item_outbound ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

            {
                {   // task1
                    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());     // 调用 MoveIt 的标准 OMPL 采样规划器
                    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();                   // 关节插值规划器，用于张开/闭合夹爪
                    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                                   // 笛卡尔路径规划器，用于末端移动

                    auto task = std::make_unique<mtc::Task>();
                    task->loadRobotModel(this->shared_from_this());
                    task->stages()->setName("item_outbound");
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
                }

                // 根据CollisionObject的id来获取物品
                geometry_msgs::msg::Pose object_pose;
                moveit_msgs::msg::CollisionObject obj;
                while (rclcpp::ok())
                {
                    std::map<std::string, moveit_msgs::msg::CollisionObject> objects = psi.getObjects({"box1"});

                    if (!objects.empty() && objects.find("box1") != objects.end())
                    {
                        // 正确获取物体对象
                        obj = objects["box1"];
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
                    task->stages()->setName("item_outbound");
                    task->setProperty("group",arm_group_name);
                    task->setProperty("eef",hand_group_name);
                    task->setProperty("ik_frame",hand_frame);

                    // 添加stage
                    {   // 初始状态
                        auto current_stage = std::make_unique<mtc::stages::CurrentState>("current state");
                        task->add(std::move(current_stage));
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
                    {   // 允许夹爪与物品碰撞
                        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
                        // 获取机械手所有具有碰撞几何的链接名称
                        auto hand_links = task->getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry();
                        stage->allowCollisions(obj.id,hand_links,true);
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
                }

                {   // task3
                    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());     // 调用 MoveIt 的标准 OMPL 采样规划器
                    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();                   // 关节插值规划器，用于张开/闭合夹爪
                    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();                                   // 笛卡尔路径规划器，用于末端移动

                    auto task = std::make_unique<mtc::Task>();
                    task->loadRobotModel(this->shared_from_this());
                    task->stages()->setName("item_outbound");
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
                        auto item_backward_stage = std::make_unique<mtc::stages::MoveRelative>("object backward",sampling_planner);
                        item_backward_stage->setGroup(arm_group_name);
                        item_backward_stage->setIKFrame(hand_frame);
                        item_backward_stage->setMinMaxDistance(0.05, 0.2/2+0.05);  //TODO:改为柜子的深度

                        geometry_msgs::msg::Vector3Stamped lift_position;
                        lift_position.header.frame_id = hand_frame;
                        lift_position.vector.z = -1;
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
                        move_down_stage->setMinMaxDistance(0,2);

                        geometry_msgs::msg::Vector3Stamped vec;
                        vec.header.frame_id = "world";
                        vec.vector.z = -1;
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
                        stage->setMinMaxDistance(0.05, 0.2/2+0.1);     //TODO:改为柜子的宽度
                        // 设置撤退方向
                        geometry_msgs::msg::Vector3Stamped vec;
                        vec.header.frame_id = hand_frame;
                        vec.vector.z = -1;
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
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),"柜子点位未被接收");
        }
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


    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    moveit::planning_interface::PlanningSceneInterface psi;
    bool is_cabinet_points_received = false;

    // 机械臂参数
    std::string robot_name;
    std::string arm_group_name;
    std::string hand_group_name;
    std::string hand_frame;
    std::map<std::pair<int,int>,std::vector<double>> cabinet_points_map;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cabinet_points_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<TestFaitinoMoveit2Control>(options);

    // 在单独的线程中执行机械臂控制
    std::thread control_thread([node]() {
        // 等待一段时间确保ROS系统完全启动
        rclcpp::sleep_for(std::chrono::seconds(3));

        // 添加碰撞物体
        node->add_collision();

        // 执行机械臂运动
        node->machinery_move();
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
