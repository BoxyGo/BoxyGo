import glob
import os
from setuptools import find_packages, setup

package_name = 'boxygo_description'

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
        (os.path.join('share', package_name, 'urdf'),
         glob.glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'urdf', 'common'),
         glob.glob(os.path.join('urdf', 'common', '*.xacro'))),
        (os.path.join('share', package_name, 'urdf', 'config'),
         glob.glob(os.path.join('urdf', 'config', '*.xacro'))),
        (os.path.join('share', package_name, 'meshes'),
         glob.glob(os.path.join('meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boxygo Team',
    maintainer_email='boxygo.mail@gmail.com',
    description='BoxyGo description package containing URDF and related files',
    license='Apache-2.0',
)