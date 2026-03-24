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
                {'voxel_size': 0.08},
                {'distance_threshold': 0.05},
                {'ransac_n': 3},
                {'num_iterations': 1500},
                {'floor_normal_min_abs_z': 0.80},
                {'wall_normal_max_abs_z': 0.50},
                {'max_walls': 6},
                {'min_floor_inliers': 500},
                {'min_wall_inliers': 300},
                {'remove_ceiling': True},
                {'ceiling_percentile': 0.98},
            ],
        )
    ])
