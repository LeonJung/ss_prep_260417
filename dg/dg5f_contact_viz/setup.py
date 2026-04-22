from setuptools import find_packages, setup

package_name = "dg5f_contact_viz"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/contact_viz.launch.py",
          "launch/contact_viz_left.launch.py",
          "launch/contact_viz_both.launch.py"]),
        ("share/" + package_name + "/config",
         ["config/contact_thresholds.yaml",
          "config/contact_thresholds_left.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jung",
    maintainer_email="jung.ryuwoon@gmail.com",
    description="DG5F per-joint contact visualization from motor current feedback.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "contact_monitor = dg5f_contact_viz.contact_monitor_node:main",
            "contact_viz = dg5f_contact_viz.contact_viz_widget:main",
        ],
    },
)
