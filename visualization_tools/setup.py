from setuptools import find_packages, setup

package_name = "visualization_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tutu",
    maintainer_email="juliwhit@umich.edu",
    description="expanded foxglove visualization suite",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "foxglove_voxel = visualization_tools.occupancy_grid_visualizer:main",
            "linear_speed = visualization_tools.gauge_message:main",
        ],
    },
)
