from setuptools import setup

package_name = "axon_system"

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
    description="System management for Axon.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "diagnostics_node = axon_system.diagnostics_node:main",
            "mode_manager_node = axon_system.mode_manager_node:main",
            "watchdog_node = axon_system.watchdog_node:main",
        ]
    },
)
