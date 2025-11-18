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
    point_cloud_extractor_node = Node(
        package='item_recognition',
        executable='point_cloud_extractor_qr',
        output='both',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('camera/depth_rect_to_color/image', 'camera/depth/image_raw'),  # 输入已将RGB图像和深度图像配准的 图像（注意是深度图向RGB图对准）
            ('camera/depth_rect_to_color/camera_info', 'camera/depth/camera_info'),      # 输入已将RGB图像和深度图像配准的 矫正信息（注意是深度图向RGB图对准）
            ('qr/qr_msg', 'qr/qr_msg')
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