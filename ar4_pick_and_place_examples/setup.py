from setuptools import setup, find_packages
import os
from glob import glob

package_name = "ar4_pick_and_place_examples"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # Include config files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="yaguan@example.com",
    description="Vision-based pick-and-place examples for the AR4 robot",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simple_pick_place_demo = ar4_pick_and_place_examples.simple_pick_place_demo:main",
            "vision_pick_place_demo = ar4_pick_and_place_examples.vision_pick_place_demo:main",
            "spawn_demo_objects = ar4_pick_and_place_examples.spawn_demo_objects:main",
            "test_gripper = ar4_pick_and_place_examples.test_gripper:main",
        ],
    },
)
