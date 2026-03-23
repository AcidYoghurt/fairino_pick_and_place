from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([

        # --- 1) ArUco 检测节点 ---
        Node(
            package='perception_layer',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            parameters=[{
                "marker_length": 0.05
            }],
            output='screen'
        ),

        # --- 2) ArUco Pose → Base坐标系转换节点 ---
        Node(
            package='perception_layer',
            executable='aruco_pose_to_base_node',
            name='aruco_pose_to_base_node',
            output='screen',
            parameters=[{
                "target_frame": "base_link"    # 或者你的机械臂 base frame 名称
            }]
        )
    ])
