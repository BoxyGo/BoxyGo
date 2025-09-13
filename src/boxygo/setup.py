from setuptools import setup, find_packages

setup(
    name='boxygo',     
    version='0.0.1',
        packages=find_packages(
        include=['boxygo', 'boxygo.*']
    ),
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'boxygo_moteus = boxygo.boxygo_moteus:main',
        ],
    },
)
