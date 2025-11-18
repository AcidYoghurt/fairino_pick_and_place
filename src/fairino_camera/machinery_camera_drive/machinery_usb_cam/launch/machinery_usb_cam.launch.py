import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_config_path = os.path.join(get_package_share_directory("machinery_usb_cam"),"config","camera_config.yaml")
    namespace=""

    # 打开摄像头
    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace=namespace,
        output="both",
        parameters=[camera_config_path]
    )

    # 矫正节点，去畸变
    rectify_node = Node(
        package="machinery_usb_cam",
        executable="rectify_node",
        output="both",
        remappings=[
            ("/image_raw",namespace+"/image_raw"),
            ('/camera_info',namespace+'/camera_info')
        ],
    )


    return LaunchDescription([
        usb_cam_node,
        rectify_node,
    ])
