import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mfrankic_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marin Frankic',
    maintainer_email='mfrankic@riteh.hr',
    description='DZ1',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'odom_subscriber = {package_name}.odom_subscriber:main',
        ],
    },
)
