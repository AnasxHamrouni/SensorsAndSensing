import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    madgwick_config = os.path.join(
        get_package_share_directory("imu_lab_ros2"),
        "config",
        "madgwick.yaml",
    )

    return LaunchDescription([
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            output="screen",
            parameters=[madgwick_config],
            remappings=[
                ("imu/data_raw", "/imu/raw"),
                ("imu/data", "/imu/data"),
            ],
        )
    ])
