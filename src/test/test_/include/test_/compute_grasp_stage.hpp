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

class WaitForObjectPose : public moveit::task_constructor::MonitoringGenerator
{
public:
    WaitForObjectPose(const std::string& name) : MonitoringGenerator(name)
    {
        setProperty("object_id", "box1");
    }

protected:
    void onNewSolution(const moveit::task_constructor::SolutionBase& /*unused*/) override {}

    void compute() override
    {
        const std::string object_id = properties().get<std::string>("object_id");
        planning_scene_monitor::LockedPlanningSceneRO scene(pipeline()->planningSceneMonitor());
        const auto& world = scene->getWorld();
        const auto& objects = world->getObject(object_id);

        if (!objects)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WaitForObjectPose"), "未找到物体: " << object_id);
            return;
        }

        const Eigen::Isometry3d& pose = objects->shape_poses_[0];
        geometry_msgs::msg::PoseStamped pose_msg;
        tf2::toMsg(pose, pose_msg.pose);
        pose_msg.header.frame_id = scene->getPlanningFrame();

        // 构造新的 PlanningScene 并发送
        planning_scene::PlanningScenePtr new_scene = scene->diff();
        auto state = std::make_shared<moveit::task_constructor::InterfaceState>(new_scene);
        state->properties().set("target_pose", pose_msg);
        this->spawn(std::move(state));
    }
};
