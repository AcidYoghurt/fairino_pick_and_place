from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # 声明可配置参数
    server_ip_arg = DeclareLaunchArgument(
        "server_ip",
        default_value="192.168.58.161",
        description="TCP 服务器 IP"
    )

    server_port_arg = DeclareLaunchArgument(
        "server_port",
        default_value="1233",
        description="TCP 服务器端口"
    )

    heartbeat_interval_arg = DeclareLaunchArgument(
        "heartbeat_interval",
        default_value="30",
        description="心跳包间隔（秒）"
    )

    device_id_arg = DeclareLaunchArgument(
        "deviceId",
        default_value="1990235585032663041",
        description="设备 ID"
    )

    device_name_arg = DeclareLaunchArgument(
        "deviceName",
        default_value="robotic_arm_01",
        description="设备名称"
    )

    # 启动节点
    tcp_client_node = Node(
        package="fairino_network_connect",
        executable="tcp_ros_node",
        output="screen",
        parameters=[
            {
                "server_ip": LaunchConfiguration("server_ip"),
                "server_port": LaunchConfiguration("server_port"),
                "heartbeat_interval": LaunchConfiguration("heartbeat_interval"),
		"deviceId": ['"', LaunchConfiguration("deviceId"), '"'],
                "deviceName": LaunchConfiguration("deviceName"),
            }
        ]
    )

    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        heartbeat_interval_arg,
        device_id_arg,
        device_name_arg,
        tcp_client_node
    ])
