from setuptools import find_packages, setup

package_name = "vertex_fleet"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tashi",
    maintainer_email="yeousunn@tashi.network",
    description="Consensus-coordinated agent library on the Tashi Vertex "
                "integration (vertex_ros2).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ledger_agent = vertex_fleet.examples.ledger_agent:main",
        ],
    },
)
