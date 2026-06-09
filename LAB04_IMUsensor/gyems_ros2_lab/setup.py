from setuptools import find_packages, setup


package_name = "gyems_ros2_lab"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/ekf_imu_wheel.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lab Student",
    maintainer_email="student@example.com",
    description="Extra-task skeleton for BLDC driver, rail controller, odometry, and EKF fusion.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gyems_motor_driver_node = gyems_ros2_lab.gyems_motor_driver_node:main",
            "rail_controller_node = gyems_ros2_lab.rail_controller_node:main",
        ],
    },
)
