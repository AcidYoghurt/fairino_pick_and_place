import os.path
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def declare_parameters():
    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value="False",
        description="是否使用仿真时间"
    )

    return [use_sim_time_param]

def cabinet_points():
    cabinet_param_config = get_package_share_directory("fairino_cabinet")

    cabinet_points_node = Node(
        package="fairino_cabinet",
        executable="cabinet_points",
        output="screen",
        parameters=[
            os.path.join(cabinet_param_config,'config','cabinet_param.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return [cabinet_points_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    cabinet_points_node = cabinet_points()

    return LaunchDescription(
        declare_parameters_node +
        cabinet_points_node
    )
