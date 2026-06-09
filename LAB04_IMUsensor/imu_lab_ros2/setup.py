from setuptools import find_packages, setup


package_name = "imu_lab_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/imu_raw.launch.py",
            "launch/madgwick.launch.py",
            "launch/imu_pipeline.launch.py",
        ]),
        (f"share/{package_name}/config", [
            "config/ekf_imu.yaml",
            "config/madgwick.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lab Student",
    maintainer_email="student@example.com",
    description="MPU6050 IMU ROS2 package for Sensors & Sensing Lab 4.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpu6050_node = imu_lab_ros2.mpu6050_node:main",
        ],
    },
)
