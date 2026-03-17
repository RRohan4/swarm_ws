from setuptools import find_packages, setup

package_name = "swarm_slam"

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
    maintainer="Sean Yang",
    maintainer_email="sean@seanyang.me",
    description="Global map merge node for swarm exploration",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_merge_node = swarm_slam.map_merge_node:main",
        ],
    },
)
