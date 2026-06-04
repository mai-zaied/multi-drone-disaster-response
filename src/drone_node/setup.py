from setuptools import find_packages, setup

package_name = 'drone_node'

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
    maintainer_email='maizaied03@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'drone_status_publisher = drone_node.drone_status_publisher:main',
        'drone_reactor = drone_node.drone_reactor:main',
        'drone_task_publisher = drone_node.drone_task_publisher:main',
        'camera_bridge_simple = drone_node.camera_bridge_simple:main',
        'victim_detector = drone_node.victim_detector:main',
        'cloud_detector = drone_node.cloud_detector:main',
    ],
},
)
