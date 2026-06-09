from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
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
        )
    ])
