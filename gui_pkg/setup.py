from setuptools import find_packages, setup

package_name = "gui_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ano",
    maintainer_email="nora611hanyafife@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "batgui = gui_pkg.batgui:main",
            "joy_to_motion = gui_pkg.joy_to_motion:main",
            "keyboard_to_twist = gui_pkg.keyboard_to_twist:main",
        ],
    },
)
