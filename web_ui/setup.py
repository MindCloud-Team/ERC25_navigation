from setuptools import find_packages, setup
import os

package_name = 'web_ui'

# Standard ROS 2 package resource index and manifest file
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(directory):
    """
    Recursively gather all files under 'directory'.
    Returns a list of file paths relative to the project root.
    """
    paths = []
    for (root, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(root, filename))
    return paths

# Include web assets under web_ui/www, web_ui/templates, web_ui/static
asset_dirs = ['www', 'templates', 'static']
for subdir in asset_dirs:
    full_path = os.path.join('web_ui', subdir)
    if os.path.exists(full_path):
        files = package_files(full_path)
        dest = os.path.join('share', package_name, subdir)
        data_files.append((dest, files))

# Include ROS2 launch files (if any)
if os.path.exists('launch'):
    launch_files = []
    for name in os.listdir('launch'):
        file_path = os.path.join('launch', name)
        if os.path.isfile(file_path):
            launch_files.append(file_path)
    data_files.append((os.path.join('share', package_name, 'launch'), launch_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=[
        'setuptools',
        'flask',
        'flask-socketio',
        'opencv-python',
        'cv-bridge',
        'numpy',
    ],
    zip_safe=True,
    maintainer='bab',
    maintainer_email='bab@todo.todo',
    description=('Custom Web-based UI for ROS2 rover control and ' 
                 'monitoring with direct ROS2 communication'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_ui_server = web_ui.web_server:main',
        ],
    },
)

