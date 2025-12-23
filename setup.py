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
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    (os.path.join('share', package_name, 'rviz'),
     glob(os.path.join('rviz', '*'))),
    (os.path.join('share', package_name, 'config'),
     glob(os.path.join('config', '*'))),
]

data_files += package_files('description')
data_files += package_files('urdf')

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=[
        'setuptools',
        'pyyaml',
        "numpy"
    ],
    zip_safe=True,
    author="Liam Bray",
    description="Roboboat autonomous navigation package",
    license="GNU GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bag = mhseals_nav.utils.bag:main",
            "test = mhseals_nav.test:main",
            "twist_converter = mhseals_nav.twist_converter:main"
        ],
    },
)
