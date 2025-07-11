from setuptools import setup
import os
from glob import glob

package_name = 'image_compression_util'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Package metadata
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Utility to compress image topics from config file',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

