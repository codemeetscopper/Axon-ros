from setuptools import setup

package_name = "axon_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/robot.launch.py",
            "launch/base.launch.py",
            "launch/audio.launch.py",
            "launch/hri.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/base.yaml",
            "config/audio.yaml",
            "config/hri.yaml",
        ]),
        ("share/" + package_name + "/config/profiles", [
            "config/profiles/real.yaml",
            "config/profiles/dev.yaml",
            "config/profiles/sim.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.local",
    description="Launch and configuration package for Axon robot bringup.",
    license="Apache-2.0",
)
