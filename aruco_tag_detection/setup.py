from setuptools import find_packages, setup

package_name = 'aruco_tag_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mindpi5',
    maintainer_email='mindpi5@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_tag_detector = aruco_tag_detection.aruco_tag_detector:main',
            'aruco_generate_marker = aruco_tag_detection.aruco_generate_marker:main'
        ],
    },
)
