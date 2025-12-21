from setuptools import setup

package_name = "axon_interfaces"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.invalid",
    description="Topic/service constants and safety configuration for Axon.",
    license="Apache-2.0",
)
