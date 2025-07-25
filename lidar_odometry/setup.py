import os
from glob import glob
from setuptools import find_packages, setup

package_name = "lidar_odometry"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.*")),
        ),
    ],
    install_requires=["setuptools", "ros2_numpy", "ctypes", "open3d"],
    zip_safe=True,
    maintainer="ibrahim",
    maintainer_email="ibrahimsalem6622005@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "lidar_odometry = lidar_odometry.lidar_odometry:main",
        ],
    },
)
