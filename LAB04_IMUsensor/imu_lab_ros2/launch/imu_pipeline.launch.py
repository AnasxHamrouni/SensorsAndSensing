import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("imu_lab_ros2")
    madgwick_config = os.path.join(package_share, "config", "madgwick.yaml")
    ekf_config = os.path.join(package_share, "config", "ekf_imu.yaml")

    return LaunchDescription([
        Node(
            package="imu_lab_ros2",
            executable="mpu6050_node",
            name="mpu6050_node",
            output="screen",
            parameters=[
                {
                    "i2c_bus": 1,
                    "i2c_address": 0x68,
                    "publish_rate_hz": 100.0,
                    "frame_id": "imu_link",
                }
            ],
        ),
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            output="screen",
            parameters=[madgwick_config],
            remappings=[
                # Madgwick defaults to imu/data_raw; lab publisher uses /imu/raw.
                ("imu/data_raw", "/imu/raw"),
                ("imu/data", "/imu/data"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_static_tf",
            # EKF needs a valid base_link <-> imu_link relationship in TF.
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config],
        ),
    ])
