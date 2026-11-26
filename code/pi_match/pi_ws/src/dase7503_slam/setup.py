from glob import glob

from setuptools import find_packages, setup

setup(
    name="dase7503_slam",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dase7503_slam"]),
        ("share/dase7503_slam", ["package.xml"]),
        ("share/dase7503_slam/launch", glob("launch/*.launch.py")),
        ("share/dase7503_slam/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="sssssteve",
    maintainer_email="90317028+sssssteve@users.noreply.github.com",
    description="ROS 2 Jazzy mapping bringup and input checks for DASE7503",
    license="All rights reserved",
    entry_points={"console_scripts": [
        "check_inputs = dase7503_slam.check_inputs:main",
        "save_map = dase7503_slam.save_map:main",
    ]},
)
