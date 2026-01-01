from setuptools import setup

package_name = "axon_hri_voice"

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
    description="HRI voice pipeline stubs.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "voice_router_node = axon_hri_voice.voice_router_node:main",
            "tts_stub_node = axon_hri_voice.tts_stub_node:main",
            "stt_stub_node = axon_hri_voice.stt_stub_node:main",
        ]
    },
)
