import os
from glob import glob
from setuptools import find_packages, setup

package_name = "local_costmap"

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
    zip_safe=True,
    maintainer="reem",
    maintainer_email="reemibraahiim1@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "local_costmap_node = local_costmap.local_costmap_node:main",
            "stereo_costmap_node = local_costmap.stereo_costmap_node:main",
        ],
    },
)
