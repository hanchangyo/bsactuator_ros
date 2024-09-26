from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bsactuator_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanc',
    maintainer_email='k9s1j7@gmail.com',
    description='ROS2 package for controlling BambooshootActuator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bsactuator_ros = bsactuator_ros.bsactuator_ros:main'
        ],
    },
)
