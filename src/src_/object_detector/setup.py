from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share",package_name,"data"), glob("data/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tobias',
    maintainer_email='tobias.haugg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = object_detector.ros_node:main',
        ],
    },
)
