import os
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

def point_cloud_extractor():
    config_file = os.path.join(get_package_share_directory("item_recognition"),'config','collision_object_manage','collision_object_size.yaml')

    point_cloud_extractor_node = Node(
        package='item_recognition',
        executable='collision_object_manage',
        output='both',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),},
            config_file
        ],
    )

    return [point_cloud_extractor_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    point_cloud_extractor_node = point_cloud_extractor()

    return LaunchDescription(
        declare_parameters_node +
        point_cloud_extractor_node
    )