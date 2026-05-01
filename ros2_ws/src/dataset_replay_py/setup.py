from setuptools import setup

package_name = "dataset_replay_py"

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
    description="KITTI replay ROS2 package.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "kitti_replay_node = dataset_replay_py.kitti_replay_node:main",
            "dump_tracks_eval_node = dataset_replay_py.dump_tracks_eval_node:main",
        ],
    },
)
