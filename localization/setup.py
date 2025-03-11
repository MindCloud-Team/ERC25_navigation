import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mwa',
    maintainer_email='mohamedwael200320@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dead_reckoning = localization.dead_reckoning:main',
            'encoder_sim = localization.encoder_sim:main',
            'dead_reckoning_sim = localization.dead_reckoning_sim:main',
        ],
    },
)
