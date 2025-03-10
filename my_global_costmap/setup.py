from setuptools import find_packages, setup

package_name = 'my_global_costmap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools' , 'nav2_costmap_2d'],
    zip_safe=True,
    maintainer='reem',
    maintainer_email='reemibraahiim1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_layer = my_global_costmap.obstacle_layer:main',
        ],
        'nav2_costmap_2d.costmap_plugins': [
            'obstacle_layer = my_global_costmap.obstacle_layer:main',
        ],
    },
)
