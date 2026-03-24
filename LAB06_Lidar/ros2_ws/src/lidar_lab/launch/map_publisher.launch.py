from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_lab',
            executable='map_publisher',
            name='map_publisher',
            output='screen',
            parameters=[
                {'map_path': '/home/nasta/Documents/GitHub/SensorsAndSensing/LAB06_Lidar/room_map_3d_colored.ply'},
                {'map_topic': '/map_3d'},
                {'frame_id': 'livox_frame'},
                {'publish_period_sec': 1.0},
            ],
        )
    ])
