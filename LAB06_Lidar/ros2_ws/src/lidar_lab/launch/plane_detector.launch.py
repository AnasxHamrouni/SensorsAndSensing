from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_lab',
            executable='plane_detector',
            name='plane_detector',
            output='screen',
            parameters=[
                {'input_topic': '/livox/lidar'},
                {'voxel_size': 0.05},
                {'distance_threshold': 0.02},
                {'ransac_n': 3},
                {'num_iterations': 1000},
                {'max_planes': 5},
                {'min_inlier_ratio': 0.05},
            ],
        )
    ])
