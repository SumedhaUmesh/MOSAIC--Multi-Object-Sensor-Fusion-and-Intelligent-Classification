from setuptools import setup

package_name = "lane_detection_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MOSAIC",
    maintainer_email="dev@example.com",
    description="Lane detection ROS2 package.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lane_detection_node = lane_detection_py.lane_detection_node:main",
        ],
    },
)
