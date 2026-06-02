from setuptools import setup

package_name = "autoracer_carmaker_sim"

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
    maintainer="Autoracer Team",
    maintainer_email="autoracer@example.com",
    description="CarMaker stage A simulation adapters for autoracer_hooke.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "carmaker_trajectory_provider = autoracer_carmaker_sim.trajectory_provider:main",
        ],
    },
)
