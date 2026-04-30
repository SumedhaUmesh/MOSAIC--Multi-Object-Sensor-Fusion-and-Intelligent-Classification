from setuptools import setup

package_name = "perception_camera_py"

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
    description="Camera perception ROS2 package.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_perception_node = perception_camera_py.camera_perception_node:main",
        ],
    },
)
