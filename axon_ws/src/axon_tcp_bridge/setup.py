from setuptools import setup

package_name = "axon_tcp_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/scripts", ["scripts/echo_server.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.local",
    description="TCP bridge architecture placeholders for Axon.",
    license="Apache-2.0",
)
