from setuptools import setup
from setuptools import find_packages

setup(
    name='bbox_display',
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/bbox_display']),
        ('share/bbox_display', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reem',
    maintainer_email='reem@example.com',
    description='BBox display node',
    license='MIT',
    tests_require=['pytest'],
    include_package_data=True, 
    entry_points={
        'console_scripts': [
            'bbox_to_marker = bbox_display.bbox_to_marker:main',
        ],
    },
)
