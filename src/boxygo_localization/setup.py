import glob
import os
from setuptools import find_packages, setup

package_name = 'boxygo_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob.glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob.glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'),
         glob.glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'maps'),
         glob.glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boxygo Team',
    maintainer_email='boxygo.mail@gmail.com',
    description='BoxyGo slam package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'realsense_imu_remap = boxygo_localization.realsense_imu_remap:main',
        ],
    },
)
