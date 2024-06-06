from setuptools import setup
import os
from glob import glob

package_name = 'mode_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 node for publishing mode based on user input',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_publisher = mode_publisher.mode_publisher_node:main'
        ],
    },
)
