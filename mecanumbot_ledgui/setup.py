from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecanumbot_ledgui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mecanumbot_ledgui.launch.py']),
        (os.path.join('share', package_name, 'configs'), glob('configs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adorjan',
    maintainer_email='adorjan@todo.todo',
    description='LED GUI',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        	'mecanumbot_ledgui = mecanumbot_ledgui.mecanumbot_ledgui:main',

        ],
    },
)
