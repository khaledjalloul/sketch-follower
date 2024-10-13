from setuptools import find_packages, setup
from glob import glob

package_name = "sketch_follower"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*launch.[pxy][yma]*")),
        (
            "share/" + package_name + "/robot_description/urdf",
            glob("robot_description/urdf/*"),
        ),
        (
            "share/" + package_name + "/robot_description/meshes",
            glob("robot_description/meshes/*"),
        ),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Khaled Jalloul",
    maintainer_email="khaled.jalloul@hotmail.com",
    description="Sketch Follower",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cursor_publisher = sketch_follower.ros.cursor_publisher",
            "controller = sketch_follower.controller",
            "teleop = sketch_follower.teleop",
        ],
    },
)
