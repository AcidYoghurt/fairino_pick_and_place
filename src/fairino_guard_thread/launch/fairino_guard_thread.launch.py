from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def declare_parameters():
    # 声明参数
    config_path_param = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([FindPackageShare('fairino_guard_thread'),'config']),
        description='config目录 路径'
    )
    return [config_path_param]

def fairino_guard_thread():
    fairino_guard_thread_node = Node(
        package='fairino_guard_thread',
        executable='fairino_guard_thread_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'fairino_guard_thread.yaml'])
        ]
    )
    return [fairino_guard_thread_node]


def generate_launch_description():
    # 节点
    declare_parameters_node = declare_parameters()
    fairino_guard_thread_node = fairino_guard_thread()

    return LaunchDescription(
        declare_parameters_node +
        fairino_guard_thread_node
    )