"""기존 LiDAR 드라이버를 건드리지 않는 통합 센싱 노드 실행."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [DeclareLaunchArgument("serial_port", description="ESP32의 /dev/serial/by-id 경로"),
                 DeclareLaunchArgument("topic", default_value="/scan"),
                 DeclareLaunchArgument("uwb_topic", default_value="/smartcart/uwb"),
                 DeclareLaunchArgument("port", default_value="8765"),
                 DeclareLaunchArgument("uwb_yaw", default_value="0"),
                 DeclareLaunchArgument("uwb_x", default_value="0"),
                 DeclareLaunchArgument("uwb_y", default_value="0")]
    return LaunchDescription(arguments + [
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", EnvironmentVariable("ROS_LOCALHOST_ONLY", default_value="1")),
        Node(package="smartcart_sensing", executable="smartcart-sensing", output="screen",
             arguments=["serve", "--mode", "ros", "--uwb-mode", "serial",
                        "--serial-port", LaunchConfiguration("serial_port"),
                        "--topic", LaunchConfiguration("topic"),
                        "--uwb-topic", LaunchConfiguration("uwb_topic"),
                        "--host", "127.0.0.1", "--port", LaunchConfiguration("port"),
                        "--uwb-yaw", LaunchConfiguration("uwb_yaw"),
                        "--uwb-x", LaunchConfiguration("uwb_x"),
                        "--uwb-y", LaunchConfiguration("uwb_y")])])
