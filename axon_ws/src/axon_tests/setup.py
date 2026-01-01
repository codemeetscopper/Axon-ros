from setuptools import setup

package_name = "axon_tests"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pytest"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.local",
    description="Unit tests for Axon packages.",
    license="Apache-2.0",
)
