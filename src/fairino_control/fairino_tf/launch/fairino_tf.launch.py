from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def declare_params():
    # 声明参数
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([FindPackageShare('fairino_description'), 'urdf']),
        description='URDF目录 路径'
    )

    config_path_arg = DeclareLaunchArgument(
        name='config_path',
        default_value=PathJoinSubstitution([FindPackageShare('fairino_tf'), 'config']),
        description='config目录 路径'
    )

    rviz_path_arg = DeclareLaunchArgument(
        name='rviz_path',
        default_value=PathJoinSubstitution([FindPackageShare('fairino_tf'), 'rviz']),
        description='rviz目录 路径'
    )

    return [urdf_path_arg,config_path_arg,rviz_path_arg]

def machinery_robot_tf(context):
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([context.launch_configurations['urdf_path'],'fairino5_v6.urdf'])
    ])

    # 节点
    machinery_robot_tf_node = Node(
        package='fairino_tf',
        executable='machinery_robot_tf',
        output='both',
        parameters=[{
        }]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',  PathJoinSubstitution([LaunchConfiguration('rviz_path'),'fairino_tf.rviz'])],
        output='screen'
    )
    return [machinery_robot_tf_node,robot_state_publisher_node,rviz_node]


def machinery_camera_tf():
    # 节点
    machinery_camera_tf_node = Node(
        package='fairino_tf',
        executable='machinery_camera_tf',
        output='both',
        parameters=[{
            'config_path': ParameterValue(LaunchConfiguration('config_path'), value_type=str)
        }]
    )

    machinery_tool_frame_node = Node(
        package='fairino_tf',
        executable='machinery_tool_frame',
        output='both',
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'tool_frame.yaml'])
        ]
    )
    return [machinery_camera_tf_node,machinery_tool_frame_node]

def generate_launch_description():

    declare_params_node = declare_params()
    machinery_camera_tf_node = machinery_camera_tf()

    return LaunchDescription(
        declare_params_node +
        machinery_camera_tf_node +
        [OpaqueFunction(function=machinery_robot_tf)]
    )
