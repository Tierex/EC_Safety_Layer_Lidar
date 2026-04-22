import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'truck_gateway'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tiemen-vr',
    maintainer_email='tiemen-vr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'truck_can_exe = truck_gateway.truck_CAN:main',
            'xbox_translator_exe = truck_gateway.xbox_translator:main'
        ],
    },
)
