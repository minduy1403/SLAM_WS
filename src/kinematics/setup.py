from setuptools import setup
import os

package_name = 'kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # thêm launch folder và file launch
        (os.path.join('share', package_name, 'launch'),
            ['launch/manual.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='tranthaiducduyduy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kinematic = kinematics.inverse_kinematic_node:main',
            'serial_driver       = kinematics.serial_driver_node:main',
        ],
    },
)
