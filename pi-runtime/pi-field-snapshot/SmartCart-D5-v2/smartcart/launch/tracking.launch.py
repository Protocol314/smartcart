"""기존 드라이버와 좌우 UWB 수신기는 별도로 두고 추적 GUI만 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def example(name):
        return PathJoinSubstitution([FindPackageShare("smartcart_sensing"), "config", name])
    return LaunchDescription([
        DeclareLaunchArgument("mounting_config", default_value=example("dual_uwb.d5.example.json")),
        DeclareLaunchArgument("uwb_config", default_value=example("uwb_estimation.example.json")),
        DeclareLaunchArgument("tracking_config", default_value=example("target_tracking.example.json")),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("status_topic", default_value="/smartcart/uwb/dual/status"),
        DeclareLaunchArgument("port", default_value="8765"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        Node(package="smartcart_sensing", executable="smartcart-sensing", output="screen", arguments=[
            "serve", "--mode", "ros", "--uwb-mode", "off", "--tracking-mode", "ros",
            "--tracking-mounting-config", LaunchConfiguration("mounting_config"),
            "--tracking-uwb-config", LaunchConfiguration("uwb_config"),
            "--tracking-config", LaunchConfiguration("tracking_config"),
            "--tracking-status-topic", LaunchConfiguration("status_topic"),
            "--topic", LaunchConfiguration("scan_topic"), "--host", "127.0.0.1",
            "--port", LaunchConfiguration("port")]),
    ])
