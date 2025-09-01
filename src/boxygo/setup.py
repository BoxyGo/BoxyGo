from setuptools import setup, find_packages

setup(
    name='boxygo',            # MUSI się zgadzać z <name> w package.xml
    version='0.0.1',
    # upewnij się, że wykryje też podpakiet scripts:
    packages=find_packages(
        include=['boxygo', 'boxygo.*']
    ),
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            # ta literałka to polecenie, którego szuka ros2 run
            'wheel_moteus = boxygo.wheel_moteus:main',
        ],
    },
)
