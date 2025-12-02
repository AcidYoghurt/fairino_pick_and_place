from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 声明参数
    config_file_name_arg = DeclareLaunchArgument(
        name='config_file_name',
        default_value='eye_in_hand_calibrate.calib',
        description='要传入的config文件名称'
    )

    # 节点
    machinery_camera_tf_node = Node(
            package='machinery_camera_tf',
            executable='machinery_camera_tf',
            output='both',
            parameters=[{
                'config_file_name': LaunchConfiguration('config_file_name')
            }]
    )

    return LaunchDescription([
        # 声明参数
        config_file_name_arg,

        # 节点
        machinery_camera_tf_node
    ])