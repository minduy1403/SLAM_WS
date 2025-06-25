from setuptools import setup

package_name = 'omnibot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
         # ament resource
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # launch files
        ('share/' + package_name + '/launch', [
            'launch/omnibot_description.launch.py',
        ]),
        # URDF (xacro) files
        ('share/' + package_name + '/urdf', [
            'urdf/omnibot.urdf.xacro',
        ]),
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
        ],
    },
)
