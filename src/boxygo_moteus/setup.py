import glob
import os
from setuptools import find_packages, setup

package_name = 'boxygo_moteus'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boxygo Team',
    maintainer_email='boxygo.mail@gmail.com',
    description='BoxyGo Moteus controller interface for controlling motors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boxygo_moteus = boxygo_moteus.boxygo_moteus:main',  # Funkcja main() w boxygo_moteus.py
        ],
    },
)
