import glob
import os
from setuptools import find_packages, setup

package_name = 'boxygo_gazebo'

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
         (os.path.join('share', package_name, 'worlds'),
         glob.glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boxygo Team',
    maintainer_email='boxygo.mail@gmail.com',
    description='BoxyGo Moteus controller interface for controlling motors',
    license='Apache-2.0',
)
