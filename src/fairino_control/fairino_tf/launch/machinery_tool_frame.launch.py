from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def declare_params():
    config_path_arg = DeclareLaunchArgument(
        name='config_path',
        default_value=PathJoinSubstitution([FindPackageShare('fairino_tf'), 'config']),
        description='config目录 路径'
    )

    return [config_path_arg]

def machinery_tool_frame():
    machinery_tool_frame_node = Node(
        package='fairino_tf',
        executable='machinery_tool_frame',
        output='both',
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'tool_frame.yaml'])
        ]
    )
    return [machinery_tool_frame_node]

def generate_launch_description():
    declare_params_node = declare_params()
    machinery_robot_tf_node = machinery_tool_frame()
    return LaunchDescription(
        declare_params_node +
        machinery_robot_tf_node
    )
