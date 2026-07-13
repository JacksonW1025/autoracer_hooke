from setuptools import find_packages, setup


package_name = "autoracer_rc_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Autoracer Team",
    maintainer_email="codex@openai.com",
    description="Thin RC sensor and vehicle adapters for the shared Autoracer core.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rc_serial_interface = autoracer_rc_adapter.rc_serial_interface:main",
            "c32_pointcloud_adapter = autoracer_rc_adapter.c32_pointcloud_adapter:main",
        ],
    },
)
