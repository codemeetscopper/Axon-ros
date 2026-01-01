from setuptools import setup

package_name = "axon_base_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.local",
    description="Serial chassis driver for Axon.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "base_node = axon_base_driver.node:main",
        ]
    },
)
