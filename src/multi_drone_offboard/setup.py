from setuptools import find_packages, setup

package_name = 'multi_drone_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maizaied03',
    maintainer_email='maizaied03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'offboard = multi_drone_offboard.offboard_control:main',
            'monitor = multi_drone_offboard.swarm_monitor:main',
            'camera_bridge = multi_drone_offboard.camera_bridge:main',
        ],
    },
)
