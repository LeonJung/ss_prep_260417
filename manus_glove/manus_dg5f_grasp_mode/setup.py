from glob import glob
import os
from setuptools import setup

package_name = "manus_dg5f_grasp_mode"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LeonJung",
    maintainer_email="jung.ryuwoon@gmail.com",
    description="Grasp-mode mux node for Manus -> DG5F teleop.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_mode_node = manus_dg5f_grasp_mode.grasp_mode_node:main",
        ],
    },
)
