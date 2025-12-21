from setuptools import setup

package_name = "axon_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/pi_core.launch.py"]),
        (f"share/{package_name}/config", ["config/pi_core.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.invalid",
    description="Launch and config for Axon bringup.",
    license="Apache-2.0",
)
