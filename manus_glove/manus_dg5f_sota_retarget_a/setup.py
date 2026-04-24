from setuptools import find_packages, setup

package_name = "manus_dg5f_sota_retarget_a"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/manus_teleop_right.launch.py",
          "launch/manus_teleop_left.launch.py",
          "launch/retarget_only.launch.py"]),
        ("share/" + package_name + "/config",
         ["config/right_hand.yaml",
          "config/left_hand.yaml"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="jung",
    maintainer_email="jung.ryuwoon@gmail.com",
    description="SOTA retargeting (A-base) Manus -> DG5F via SLSQP + URDF FK.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "retarget_node = manus_dg5f_sota_retarget_a.retarget_node:main",
        ],
    },
)
