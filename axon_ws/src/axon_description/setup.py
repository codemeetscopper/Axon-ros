from setuptools import setup

package_name = "axon_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", ["urdf/axon.urdf.xacro"]),
        ("share/" + package_name + "/config", ["config/frames.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.local",
    description="Robot description for Axon.",
    license="Apache-2.0",
)
