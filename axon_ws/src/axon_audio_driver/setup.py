from setuptools import setup

package_name = "axon_audio_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axon Dev",
    maintainer_email="dev@axon.local",
    description="Microphone and speaker nodes for Axon.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mic_node = axon_audio_driver.mic_node:main",
            "speaker_node = axon_audio_driver.speaker_node:main",
        ]
    },
)
