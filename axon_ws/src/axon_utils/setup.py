from setuptools import setup

package_name = "axon_utils"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.local",
    description="Shared utilities for the Axon workspace.",
    license="Apache-2.0",
)
