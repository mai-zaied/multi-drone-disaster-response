from setuptools import setup

package_name = 'decision_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Doaa',
    maintainer_email='doaa@example.com',
    description='Threat decision logic and coordination node for UAV swarm disaster response.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_detection_publisher = decision_node.fake_detection_publisher:main',
        ],
    },
)
