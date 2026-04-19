from setuptools import find_packages, setup

package_name = "manus_dg5f_retarget"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/manus_teleop_right.launch.py"]),
        ("share/" + package_name + "/config",
         ["config/right_hand_calib.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jung",
    maintainer_email="jung.ryuwoon@gmail.com",
    description="Manus glove -> DG5F retargeting.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "retarget_node = manus_dg5f_retarget.retarget_node:main",
        ],
    },
)
