from setuptools import find_packages, setup

package_name = 'uart_odom'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/uart_odom']),
        ('share/uart_odom', ['package.xml']),
        ('share/uart_odom/launch', [
            'launch/uart_odom.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@example.com',
    description='UART odometry node for SLAM and Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'uart_odom_node = uart_odom.uart_odom_node:main',
        ],
    },
)
