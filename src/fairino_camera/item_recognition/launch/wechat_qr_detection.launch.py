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

    wechat_qr_detection_node = Node(
        package='item_recognition',
        executable='wechat_qr_detection',
        output='both',
        parameters=[{
            'test': test_mode
        }],
        remappings=[
            ('color_image_rect', 'camera/color/image_raw'),
            ('qr/image','qr/image'),
            ('qr_msg','/qr_msg')
        ],
    )

    return LaunchDescription([
        declare_test_mode_cmd,
        wechat_qr_detection_node
    ])