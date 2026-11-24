"""명시적인 데이터 포트 설정으로만 좌우 UWB 수신기를 시작한다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="확인한 좌우 USB 배정과 장착 JSON의 절대 경로"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", EnvironmentVariable("ROS_LOCALHOST_ONLY", default_value="1")),
        Node(package="smartcart_sensing", executable="smartcart-uwb-ros", output="screen",
             arguments=["--config", LaunchConfiguration("config")]),
    ])
