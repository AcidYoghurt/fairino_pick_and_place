from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,Command
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# 测试
## 物品入柜  抓取
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"pick_up\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_store\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"

## 物品入柜  放置
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"place\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_store\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"

## 物品出柜  抓取
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"pick_up\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_outbound\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"

## 物品出柜  放置
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"place\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_outbound\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"

## 演示demo  张开夹爪
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"open_gripper\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{}}'}"

## 演示demo  关闭夹爪
# ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"close_gripper\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{}}'}"


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

    machinery_camera_tf_node = Node(
        package='fairino_tf',
        executable='machinery_camera_tf',
        output='both',
        parameters=[{
            'config_path': ParameterValue(PathJoinSubstitution([LaunchConfiguration('config_path'),'fairino_control']), value_type=str),
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

    fairino_ros2_cmd_server_node = Node(
        package="fairino_hardware",
        executable="ros2_cmd_server",
        output="screen"
    )

    ros2sdk_control_node = Node(
        package="fairino_ros2sdk_control",
        executable="fairino_ros2sdk_control",
        output="screen",
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'fairino_control', 'custom_points_config.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return [fairino_ros2_cmd_server_node,ros2sdk_control_node,machinery_robot_tf_node,robot_state_publisher_node,machinery_camera_tf_node,machinery_tool_frame_node,rviz_node]

def fairino_camera():
    # 相机驱动
    orbbec_camera_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('orbbec_camera'),
            'launch',
            'gemini.launch.xml'
        ])]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_depth': 'False',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30'
        }.items()
    )

    # 感知层节点
    camera_node = Node(
        package="perception_layer",
        executable="camera_control_node",
        name="camera_control_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'fairino_camera', 'aruco_detection_params.yaml'])
        ]
    )

    Aruco_node = Node(
        package='perception_layer',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'fairino_camera', 'aruco_detection_params.yaml'])
        ],
        output='screen'
    )

    Aruco_pose = Node(
        package='perception_layer',
        executable='aruco_pose_to_base_node',
        name='aruco_pose_to_base_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'fairino_camera', 'aruco_detection_params.yaml'])
        ]
    )

    return [orbbec_camera_node,camera_node,Aruco_node,Aruco_pose]

def fairino_network_connect():
    tcp_client_node = Node(
        package="fairino_network_connect",
        executable="tcp_ros_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'fairino_network_connect', 'tcp_client_node.yaml'])
        ]
    )

    # 视频流节点
    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        output="screen",
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'),'fairino_network_connect', 'web_video_server_node.yaml'])
        ]
    )

    return [tcp_client_node,web_video_server]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    fairino_camera_node = fairino_camera()
    fairino_network_connect_node = fairino_network_connect()

    return LaunchDescription(
        declare_parameters_node +
        [OpaqueFunction(function=fairino_control)] +
        fairino_camera_node +
        fairino_network_connect_node
    )