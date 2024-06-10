from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'visual'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')), 
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='Roel de Vries',
    maintainer_email='r.d.vries1@student.tue.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera = visual.camera:main",
            "gesture = visual.gesture:main",
            "decomp_test = visual.decomp_test:main",
        ],
    },
)
