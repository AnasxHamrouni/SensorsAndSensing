from setuptools import setup

package_name = 'lidar_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/plane_detector.launch.py', 'launch/map_publisher.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nasta',
    maintainer_email='hamrounianascontact@gmail.com',
    description='Lab 06 LiDAR real-time wall/floor segmentation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'plane_detector = lidar_lab.plane_detector_node:main',
            'map_publisher = lidar_lab.map_publisher_node:main',
        ],
    },
)
