from setuptools import find_packages, setup

package_name = "dg5f_calib_wizard"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="jung",
    maintainer_email="jung.ryuwoon@gmail.com",
    description="CLI wizard to auto-calibrate manus_dg5f_sota_retarget_a yaml params.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "calib_wizard = dg5f_calib_wizard.wizard:main",
        ],
    },
)
