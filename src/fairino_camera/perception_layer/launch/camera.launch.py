from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # --- 1. 基础参数配置 (保持不变) ---
    save_directory_arg = DeclareLaunchArgument(
        "save_directory",
        default_value="out/camera_output",
        description="照片与视频保存目录"
    )

    upload_url_arg = DeclareLaunchArgument(
        "upload_url",
        default_value="http://192.168.58.161:8090/device_file/upload",
        description="文件上传接口地址"
    )

    #aruco码宽度（重要，用于深度运算）
    marker_length_arg = DeclareLaunchArgument(
        'marker_length', 
        default_value='0.05', 
        description='ArUco marker side length (m)'
    )


    # --- 2. 优化参数，用于滤波与性能优化 ---
    # 图像滤波参数（用于减少偏移与抖动），出于延迟性需求，建议设置大一些
    smoothing_alpha_arg = DeclareLaunchArgument(
        'smoothing_alpha', 
        default_value='0.6', 
        description='Smoothing factor for pose filtering (0-1, higher=smoother)'
    )


    #图像帧率窗口判断（需要多少帧确保货物稳定）
    stability_window_size_arg = DeclareLaunchArgument(
        'stability_window_size', 
        default_value='3', 
        description='Threshold for pose stability check (frames)'
    )




    #图像稳定性距离判断（用于确保要抓取货物不会发生瞬时偏移）
    stability_position_threshold_arg = DeclareLaunchArgument(
        'stability_position_threshold', 
        default_value='0.05', 
        description='Threshold for pose stability check (m)'
    )


    #节约资源参数（用于减少解析aruco码资源）
    detect_interval_arg = DeclareLaunchArgument(
        'detect_interval', 
        default_value='0.1', #默认10hz 
        description='Enable resource saving mode '
    )


    # --- 3. ArUco 碰撞检测相关参数 = ---
    
    # 货物缓冲距离：排除货物本体和5cm最小净空 (0.07m > 0.06m)
    cargo_buffer_arg = DeclareLaunchArgument(
        'x_axis_cargo_buffer_m', 
        default_value='0.02', 
        description='Buffer distance from tag edge to start collision check (m)'
    )

    # 夹爪总检测范围 (0.035 + 0.10 + 0.02 = 0.155m)
    grip_allowance_arg = DeclareLaunchArgument(
        'x_axis_grip_allowance_m', 
        default_value='0.045', # 调整：确保覆盖夹爪所需的 2cm 额外空间
        description='Total detection distance along X-axis (m)'
    )

    # Y 轴检测半宽度（涵盖7cm货物 + 冗余）
    y_width_arg = DeclareLaunchArgument(
        'y_axis_check_half_width_m', 
        default_value='0.03', 
        description='Half-width of detection zone along Y-axis (m)'
    )

    # 深度相似容差（用于侧面检测，增加容错）
    depth_tol_arg = DeclareLaunchArgument(
        'depth_similarity_tolerance_m', 
        default_value='0.05', 
        description='Tolerance for depth similarity check (m)'
    )

    front_margin_arg = DeclareLaunchArgument(
        'obstacle_in_front_margin_m', 
        default_value='0.05', 
        description='Margin to detect obstacles in front of the tag (m)'
    )


    # --- 4. 节点定义  ---

    camera_node = Node(
        package="perception_layer",             
        executable="camera_control_node",    
        name="camera_control_node",
        output="screen",
        parameters=[
            {
                "save_directory": LaunchConfiguration("save_directory"),
                "upload_url": LaunchConfiguration("upload_url"),
            }
        ]
    ) 

    Aruco_node = Node(
        package='perception_layer',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        parameters=[{ 
            "marker_length": LaunchConfiguration('marker_length'),
            "smoothing_alpha": LaunchConfiguration('smoothing_alpha'),
            "stability_window_size": LaunchConfiguration('stability_window_size'),
            "stability_position_threshold": LaunchConfiguration('stability_position_threshold'),
            "detect_interval": LaunchConfiguration('detect_interval'),
            
        }], 
        output='screen'
    ) 

    Aruco_pose = Node(
        package='perception_layer',
        executable='aruco_pose_to_base_node',
        name='aruco_pose_to_base_node',
        output='screen',
        parameters=[{
            "target_frame": "base_link"
        }]
    )
    
    return LaunchDescription([
        # 参数声明
        save_directory_arg,
        upload_url_arg,


        marker_length_arg,
        smoothing_alpha_arg,
        stability_window_size_arg,
        stability_position_threshold_arg,
        detect_interval_arg,


        cargo_buffer_arg,
        grip_allowance_arg,
        y_width_arg,
        depth_tol_arg,
        front_margin_arg,
        


        # 节点启动
        Aruco_node,
        Aruco_pose,
        camera_node
    ])
