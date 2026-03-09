from setuptools import setup

package_name = 'pkg_Anas'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nasta',
    maintainer_email='hamrounianascontact@gmail.com',
    description='Fake encoder + driver',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_encoder = pkg_Anas.fake_encoder:main',
            'encoder_driver = pkg_Anas.encoder_driver:main',
	    'wheel_odom = pkg_Anas.wheel_odom:main',
        ],
    },
)
