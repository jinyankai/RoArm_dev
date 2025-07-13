from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jyk',
    maintainer_email='2107740571@qq.com',
    description='ROS2 package for YOLO detection with depth localization.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = yolo_detector.yolo_detector_node:main',
        ],
    },
)
