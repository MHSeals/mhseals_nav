import os
from glob import glob
from setuptools import find_packages, setup

package_name = "mhseals_nav"

def package_files(directory):
    paths = []
    for path, _, files in os.walk(directory):
        for file in files:
            full_path = os.path.join(path, file)
            install_path = os.path.join(
                'share',
                package_name,
                path
            )
            paths.append((install_path, [full_path]))
    return paths

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'behavior_trees'), 
     glob(os.path.join('mhseals_nav', 'behavior_trees', '*'))),
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    (os.path.join('share', package_name, 'rviz'),
     glob(os.path.join('rviz', '*'))),
    (os.path.join('share', package_name, 'config'),
     [path for path in glob(os.path.join('config', '*')) if os.path.isfile(path)]),
]

data_files += package_files('description')
data_files += package_files('urdf')
data_files += package_files(os.path.join('config', 'tasks'))
data_files += package_files(os.path.join('config', 'fcu'))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=[
        'setuptools',
        'pyyaml',
        "numpy", "websocket-client"
    ],
    zip_safe=True,
    author="Liam Bray",
    description="Roboboat autonomous navigation package",
    license="GNU GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "twist_converter = mhseals_nav.twist_converter:main",
            "lidar_flattener = mhseals_nav.lidar_flattener:main",
            "object_tracker = mhseals_nav.object_tracker:main",
            "thruster_allocation = mhseals_nav.thruster_allocation:main",
            "zed_detections_converter = mhseals_nav.zed_detections_converter:main",
            "legacy_buoy_detector = mhseals_nav.legacy_buoy_detector:main",
            "lidar_relay = mhseals_nav.lidar_relay:main",
            "mavros_odometry = mhseals_nav.mavros_odometry:main",
            "mavros_setup = mhseals_nav.mavros_setup:main",
            "run_task = mhseals_nav.tasks.interface:main",
        ],
    },
)
