from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明launch参数
    test_mode = LaunchConfiguration('test', default='false')

    # 声明launch参数，可以在运行时通过命令行指定
    declare_test_mode_cmd = DeclareLaunchArgument(
        'test',
        default_value='False',
        description='是否启用测试模式'
    )

    pose_estimator_node = Node(
        package='perception_layer',
        executable='pose_estimator',
        output='both',
        parameters=[{
            'test': test_mode
        }],
        remappings=[
            # 订阅相机内参
            ('camera/depth_rect_to_color/camera_info', '/camera/depth/camera_info'),
            # 订阅二维码消息
            ('qr/qr_msg', 'qr/qr_msg'),
            # 订阅深度图像
            ('camera/depth_rect_to_color/image', '/camera/depth/image_raw'),
            # 发布世界坐标系下的位姿
            ('qr/item_pose_world', 'qr/item_pose_world')
        ],
    )

    return LaunchDescription([
        declare_test_mode_cmd,
        pose_estimator_node
    ])
