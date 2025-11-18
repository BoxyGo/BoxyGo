import glob
import os
from setuptools import find_packages, setup

package_name = 'boxygo_controllers'

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

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boxygo Team',
    maintainer_email='boxygo.mail@gmail.com',
    description='BoxyGo controllers package containing controller configurations',
    license='Apache-2.0',
)
