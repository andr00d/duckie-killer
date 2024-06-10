from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')), 
        (os.path.join('share', package_name, 'sounds'), glob('sounds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
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
            "Statehandler = navigation.statehandler:main",
            "rover = navigation.rover:main",

            "guard = navigation.guard:main",
            "home = navigation.home:main",
            "surveillance = navigation.surveillance:main",
        ],
    },
)
