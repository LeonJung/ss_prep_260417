from glob import glob
import os
from setuptools import setup

package_name = "dg5f_mujoco_sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LeonJung",
    maintainer_email="jung.ryuwoon@gmail.com",
    description="MuJoCo kinematic sim of DG5F with Flask MJPEG viewer.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_view = dg5f_mujoco_sim.sim_view:main",
        ],
    },
)
