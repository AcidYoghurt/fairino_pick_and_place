from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,Command
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def declare_parameters():
    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value="False",
        description="是否使用仿真时间"
    )

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([FindPackageShare('fairino_description'), 'urdf']),
        description='URDF目录 路径'
    )

    config_path_param = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([FindPackageShare('fairino_bringup'),'config']),
        description='config目录 路径'
    )

    rviz_path_param = DeclareLaunchArgument(
        'rviz_path',
        default_value=PathJoinSubstitution([FindPackageShare('fairino_bringup'),'rviz']),
        description='rviz目录 路径'
    )

    return [use_sim_time_param,urdf_path_arg,config_path_param,rviz_path_param]

def fairino_control(context):
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([context.launch_configurations['urdf_path'],'fairino5_v6.urdf'])
    ])

    # 维护TF树
    machinery_robot_tf_node = Node(
        package='fairino_tf',
        executable='machinery_robot_tf',
        output='both',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    machinery_tool_frame_node = Node(
        package='fairino_tf',
        executable='machinery_tool_frame',
        output='both',
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'fairino_control','tool_frame.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',  PathJoinSubstitution([LaunchConfiguration('rviz_path'),'fairino_tf.rviz'])],
        output='screen'
    )

    # 法奥机械臂ros2sdk控制
    fairino_ros2_cmd_server_node = Node(
        package="fairino_hardware",
        executable="ros2_cmd_server",
        output="screen"
    )

    return [fairino_ros2_cmd_server_node,machinery_robot_tf_node,robot_state_publisher_node,machinery_tool_frame_node,rviz_node]

def fairino_camera():
    # 相机驱动
    orbbec_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'gemini_330_series.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_depth': 'False',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30'
        }.items()
    )

    # Aruco码节点
    aruco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('aruco_ros'),
                'launch',
                'single.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'marker_id': '50',
            'marker_size': '0.1',
            'marker_frame': 'aruco_marker_frame',
            'reference_frame': '',
            'corner_refinement': 'LINES',
        }.items()
    )

    # 手眼标定rqt（会报错，只是为了激活rqt插件）
    easy_handeye_rqt_node = Node(
        package='easy_handeye2',
        executable='rqt_calibrator.py',
        name='rqt_calibrator',
        arguments=['--force-discover'],
        output='log'
    )

    # 手眼标定节点
    easy_handeye_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('easy_handeye2'),
                'launch',
                'calibrate.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'name': 'eye_in_hand_calibrate',
            'calibration_type': 'eye_in_hand',
            'tracking_base_frame': 'camera_link',
            'tracking_marker_frame': 'aruco_marker_frame',
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'wrist3_link'
        }.items()
    )

    return [orbbec_camera_node,aruco_node,easy_handeye_rqt_node,easy_handeye_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    fairino_camera_node = fairino_camera()
    fairino_control_node = [OpaqueFunction(function=fairino_control)]

    return LaunchDescription(
        declare_parameters_node +
        fairino_control_node +
        fairino_camera_node
    )