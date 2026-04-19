from setuptools import find_packages, setup

package_name = "manus_glove_sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jung",
    maintainer_email="jung.ryuwoon@gmail.com",
    description="Manus glove simulator (Qt slider publisher).",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "sim_glove = manus_glove_sim.manus_sim_slider:main",
        ],
    },
)
