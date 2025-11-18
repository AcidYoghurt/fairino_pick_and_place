from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def declare_parameters():
    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value="False",
        description="是否使用仿真时间"
    )

    robot_name_param = DeclareLaunchArgument(
        'robot_name',
        default_value='fairino5',
        description="机器人名称"
    )

    return [robot_name_param,use_sim_time_param]

def mtc_node(context):
    robot_name = context.launch_configurations['robot_name']
    moveit_config = MoveItConfigsBuilder(robot_name=robot_name, package_name=robot_name + "_v6_moveit2_config").to_dict()

    mtc_node = Node(
        package="test_",
        executable="test_faiirino_mtc_control",
        output="screen",
        parameters=[
            moveit_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return [mtc_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()

    return LaunchDescription(
        declare_parameters_node +
        [OpaqueFunction(function=mtc_node)]
    )