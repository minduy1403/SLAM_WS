from setuptools import setup
import os
from glob import glob

package_name = 'slam_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # index để ros2 launch tìm package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # cài launch file vào share/slam_launch/launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='tranthaiducduyduy@gmail.com',
    description='Launch SLAM pipeline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    },
)
