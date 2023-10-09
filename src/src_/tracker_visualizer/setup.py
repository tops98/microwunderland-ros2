from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'tracker_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch/'),glob('launch/*launch.[pxy][yam]*')),
        (os.path.join('share',package_name,'config/'),glob('config/*.[yam]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='tobias.haugg@haw-hamburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_visualizer = tracker_visualizer.ros_node:main'
        ],
    },
)
