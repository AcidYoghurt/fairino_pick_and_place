from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def declare_params():
  urdf_path_arg = DeclareLaunchArgument(
    'urdf_path',
    default_value=PathJoinSubstitution([FindPackageShare('fairino_description'), 'urdf', 'fairino5_v6.urdf']),
    description='URDF 文件路径'
  )

  rviz_path_arg = DeclareLaunchArgument(
    'rviz_path',
    default_value=PathJoinSubstitution([FindPackageShare('fairino_description'), 'rviz', 'display.rviz']),
    description='rviz 文件路径'
  )

  return [urdf_path_arg,rviz_path_arg]

def launch_robot_description(context):
  robot_description_content = Command([
    'xacro ', context.launch_configurations['urdf_path']
  ])

  joint_state_publisher_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui'
  )

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{
      'robot_description': ParameterValue(robot_description_content, value_type=str)
    }]
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', LaunchConfiguration('rviz_path')],
    output='screen'
  )

  return [joint_state_publisher_gui_node,robot_state_publisher_node,rviz_node]

def generate_launch_description():
  declare_params_node = declare_params()

  return LaunchDescription(
    declare_params_node +
    [OpaqueFunction(function=launch_robot_description)]
  )