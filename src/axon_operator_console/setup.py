from setuptools import setup

package_name = "axon_operator_console"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.ui"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.invalid",
    description="Operator console UI for Axon.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "operator_console = axon_operator_console.console:main",
        ]
    },
)
