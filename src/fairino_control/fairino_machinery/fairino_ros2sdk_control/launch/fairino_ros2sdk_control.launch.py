from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 测试
## 物品入柜  抓取
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"pick_up\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_store\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"

## 物品入柜  放置
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"place\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_store\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"

## 物品出柜  抓取
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"pick_up\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_outbound\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"

## 物品出柜  放置
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"place\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_outbound\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"

## 演示demo  张开夹爪
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"open_gripper\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{}}'}"

## 演示demo  关闭夹爪
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"close_gripper\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{}}'}"


def declare_parameters():
    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value="False",
        description="是否使用仿真时间"
    )

    config_path_param = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([FindPackageShare('fairino_ros2sdk_control'),'config']),
        description='config目录 路径'
    )

    return [use_sim_time_param,config_path_param]

def ros2sdk_control():
    fairino_ros2_cmd_server_node = Node(
        package="fairino_hardware",
        executable="ros2_cmd_server",
        output="screen"
    )

    ros2sdk_control_node = Node(
            package="fairino_ros2sdk_control",
            executable="fairino_ros2sdk_control",
            output="screen",
            parameters=[
                PathJoinSubstitution([LaunchConfiguration('config_path'), 'custom_points_config.yaml']),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        )

    return [fairino_ros2_cmd_server_node,ros2sdk_control_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    ros2sdk_control_node = ros2sdk_control()

    return LaunchDescription(
        declare_parameters_node +
        ros2sdk_control_node
    )