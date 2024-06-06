from setuptools import setup

package_name = 'yolo_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'torch', 'torchvision', 'opencv-python'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 node for YOLO object detection',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_detector.yolo_detector_node:main'
        ],
    },
)

