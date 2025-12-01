from setuptools import find_packages, setup
from glob import glob

package_name = 'mecanumbot_sensorprocess_smart'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',  glob('config/*')),
        ('share/' + package_name + '/models',  glob('models/*')),
        ('share/' + package_name + '/launch',  glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Csenge Hubay',
    maintainer_email='csengehubay@gmail.com',
    description='Functions for extracting and processing data from mecanumbot sensors',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mecanumbot_lidar_detect_people = mecanumbot_sensorprocess_smart.mecanumbot_lidar_detect_people:main',
        ],
    },
)
