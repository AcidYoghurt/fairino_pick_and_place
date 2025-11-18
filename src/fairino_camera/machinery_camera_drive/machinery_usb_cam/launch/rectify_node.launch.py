import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_config_path = os.path.join(get_package_share_directory("machinery_usb_cam"),"config","camera_config.yaml")
    namespace="/camera/color"

    # 矫正节点，去畸变
    rectify_node = Node(
        package="machinery_usb_cam",
        executable="rectify_node",
        output="both",
        remappings=[
            ("/image_raw",namespace+"/image_raw"),
            ('/camera_info',namespace+'/camera_info'),
            ('/image_rect',namespace+'/image_rect')
        ],
    )

    # image_proc_node = Node(
    #     package="image_proc",
    #     executable="image_proc",
    #     output="both",
    #     remappings=[
    #         ('/camera_info',namespace+'/camera_info'),
    #         ('/image',namespace+'/image_raw'),
    #     ],
    # )

    return LaunchDescription([
        rectify_node,
    ])
