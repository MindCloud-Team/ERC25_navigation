import os
from glob import glob
from setuptools import find_packages, setup

package_name = "global_planner"

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
    maintainer="ano",
    maintainer_email="ano@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "global_planner = global_planner.global:main",
        ],
    },
)
