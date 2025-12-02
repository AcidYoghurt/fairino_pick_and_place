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

def pose_qr_launch():
    pose_qr = Node(
        package='perception_layer',
        executable='pose_qr',
        output='both',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('qr/qr_msg', 'qr/qr_msg'),
            ('qr/item_pose_world', 'qr/item_pose_world'),
            ('item/create_collision', 'item/create_collision'),
            ('item/itemMsg_trigger', 'item/itemMsg_trigger')
        ],
    )

    return [pose_qr]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    pose_qr_node = pose_qr_launch()

    return LaunchDescription(
        declare_parameters_node +
        pose_qr_node
    )
