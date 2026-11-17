from glob import glob

from setuptools import find_packages, setup

package_name = "ur_inspection_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cameron Owens",
    maintainer_email="cameronallanowens@gmail.com",
    description="Launch files and checks for the UR + Zivid inspection cell.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "verify_m1 = ur_inspection_bringup.verify_m1:main",
        ],
    },
)
