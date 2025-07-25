import os
from glob import glob
from setuptools import find_packages, setup

package_name = "aruco_tag_detection"

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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mindpi5",
    maintainer_email="mindpi5@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "aruco_tag_detector = aruco_tag_detection.aruco_tag_detector:main",
            "aruco_generate_marker = aruco_tag_detection.aruco_generate_marker:main",
        ],
    },
)
